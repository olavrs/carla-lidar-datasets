import numpy as np
import pptk
import os

def get_ground_truth(sim, path):
	path = f'{path}/ground_truth/params.npy'
	return np.squeeze(np.load(path))

def get_lidar_data(sim, frame, n_lidars, path):
	path = f'{path}/converted/frame_{frame}'
	lidar_data = []
	for i in range(n_lidars):
		lidar_data.append(
			np.load(f'{path}/{i}_{frame}.npy')
		)
	return lidar_data

def cartesian2spherical(pcloud):
	xy = pcloud[0,:]**2 + pcloud[1,:]**2
	r = np.sqrt(xy + pcloud[2,:]**2)
	phi = np.arctan2(pcloud[1,:], pcloud[0,:])
	theta = np.arctan2(np.sqrt(xy), pcloud[2,:])
	pcloud = np.array([r, phi, theta])
	return pcloud

def spherical2cartesian(pcloud):
	rsintheta = pcloud[0,:]*np.sin(pcloud[2,:])
	x = rsintheta*np.cos(pcloud[1,:])
	y = rsintheta*np.sin(pcloud[1,:])
	z = pcloud[0,:]*np.cos(pcloud[2,:])
	pcloud = np.array([x, y, z])
	return pcloud


def noise(pclouds, r_std_min, r_std_max, phi_std, theta_std, lidar_range):
	r_ratio = (r_std_max - r_std_min) / lidar_range

	noisy_pclouds = []
	for pcloud in pclouds:
		N = pcloud.shape[1]
		pcloud_spherical = cartesian2spherical(pcloud)
		r_stds = r_ratio * pcloud_spherical[0,:] + r_std_min
		pcloud_spherical[0,:] = np.array([np.random.normal(pcloud_spherical[0,i], r_stds[i]) for i in range(N)])
		pcloud_spherical[1,:] += np.random.normal(0, phi_std, size=(N))
		pcloud_spherical[2,:] += np.random.normal(0, theta_std, size=(N))
		noisy_pclouds.append(spherical2cartesian(pcloud_spherical))

	return noisy_pclouds

def create_colors(pclouds):
	# Create different colors for each lidar
	clr = []
	for i, pcloud in enumerate(pclouds):
		clr.append(np.full(pcloud.shape[1], i))
	clr = np.hstack(tuple(clr))
	return clr

def compute_rotation_matrix(angs, rot_order='rpy'):
	""" Create a rotation matrix with the given roll, pitch and yaw angles (angs), 
		using the given rotation order (rot_order).
	"""
	roll, pitch, yaw = angs
	Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
	Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
	Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
	if rot_order == 'rpy':
		return Rz @ Ry @ Rx
	else:
		return Rx @ Ry @ Rz

def extract_motion_params(params, n_lidars):
	""" Extract translation vector and rotation matrix from params. """
	n_dof = 6
	t = []
	R = []
	for i in range(n_lidars):
		s = i * n_dof
		e = s + 3
		t.append(params[s:e].reshape(-1,1))
		R.append(compute_rotation_matrix(params[s+3:e+3]))
	return t, R

def create_single_point_cloud(params, data, n_lidars):
	""" Create a point cloud as a numpy array of the data using the given params. """
	# Extract motion parameters from params
	t, R = extract_motion_params(params, n_lidars)
	# Apply motion to lidar data
	point_cloud = [R[i] @ data[i] + t[i] for i in range(n_lidars)]
	point_cloud = np.hstack(tuple(point_cloud))
	return point_cloud

def main():
	root = os.path.dirname(os.path.abspath(__file__))

	sim = ''
	while sim not in list(range(0,14)):
		sim = int(input('Which simulation? (0-13): '))

	frame = 5
	lidar_range = 200
	angular_resolution = 0.3
	n_lidars = 4
	path = f'{root}/sim_{sim}/'

	if sim in [6, 10]:
		angular_resolution = 0.1

	if sim in [7, 11]:
		angular_resolution = 0.5

	if sim in [8, 12]:
		n_lidars = 2

	if sim in [9, 13]:
		n_lidars = 6

	pclouds = get_lidar_data(sim, frame, n_lidars, path)
	ground_truth = get_ground_truth(sim, path)

	add_noise = ''
	while add_noise not in ['y', 'n']:
		add_noise = input('Add noise? (y/n): ')

	if (add_noise == 'y'):
		r_std_min = 0.012
		r_std_max = 0.035
		phi_std = np.deg2rad(angular_resolution) * 0.05
		theta_std = np.deg2rad(angular_resolution) * 0.05
		
		pclouds = noise(pclouds, r_std_min, r_std_max, phi_std, theta_std, lidar_range)

	clr = create_colors(pclouds)
	pcloud = create_single_point_cloud(ground_truth, pclouds, n_lidars)

	v = pptk.viewer(pcloud.T, clr)
	input('Press any key to close the viewer...')
	v.close()

if __name__ == '__main__':
	main()