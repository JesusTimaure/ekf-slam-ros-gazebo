#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.linalg import block_diag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseArray
from sklearn.cluster import DBSCAN
from circle_fit import taubinSVD
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import TransformStamped
import tf
from tf.transformations import euler_from_quaternion
import tf.transformations as tft

class EKFSLAM:
	def __init__(self):
		rospy.init_node('ekf_slam_node', anonymous=True)
		initial_robot_pose = np.array([[-2.0], [-0.5], [0.0]])
		known_landmarks = np.array([
			[-1.0, 0.0]])
		num_landmarks = known_landmarks.shape[0]
		landmarks_vector = known_landmarks.reshape(-1, 1)
		
		# Robot and landmarks's state
		self.mu = np.vstack([initial_robot_pose, landmarks_vector])
		
		self.state_dim = self.mu.shape[0]
		
		robot_cov = np.eye(3) * 0.01
		landmark_cov = np.eye(2 * num_landmarks) * 0.01

		self.sigma = np.block([
			[robot_cov, np.zeros((3, 2*num_landmarks))],
			[np.zeros((2*num_landmarks, 3)), landmark_cov]
		])
		
		self.landmark_indices = list(range(num_landmarks))
		
		self.debug = True
		self.latest_odom = None
		self.latest_scan = None


		# Model noise (Q) and Measurement noise (R)
		self.Q = np.diag([0.05, 0.05, np.deg2rad(2)])**2
		self.R = np.diag([0.1, np.deg2rad(5)])**2
		
		# Ros subscribers and publishers
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		
		MAX_LANDMARKS = 15
		self.landmark_pubs = [
			rospy.Publisher(f'/landmark_{i}_pose_cov', PoseWithCovarianceStamped, queue_size=10)
			for i in range(MAX_LANDMARKS)  # Use the max number of landmarks you support
		]

		self.marker_pub = rospy.Publisher('/landmark_markers', MarkerArray, queue_size=10)
		self.pose_pub = rospy.Publisher('/ekf_estimated_pose', PoseWithCovarianceStamped, queue_size=10)

		

	def odom_callback(self, msg):
		self.latest_odom = msg
	
	def scan_callback(self, msg):
		max_range = 1.5
		
		filtered_ranges = []
		for r in msg.ranges:
			if np.isinf(r) or np.isnan(r) or r > max_range:
				filtered_ranges.append(float('inf'))  # or max_range if you prefer
			else:
				filtered_ranges.append(r)
		
		# Create a new LaserScan message or modify the existing one
		filtered_msg = LaserScan()
		filtered_msg.header = msg.header
		filtered_msg.angle_min = msg.angle_min
		filtered_msg.angle_max = msg.angle_max
		filtered_msg.angle_increment = msg.angle_increment
		filtered_msg.time_increment = msg.time_increment
		filtered_msg.scan_time = msg.scan_time
		filtered_msg.range_min = msg.range_min
		filtered_msg.range_max = max_range  # limit the max range here
		filtered_msg.ranges = filtered_ranges
		filtered_msg.intensities = msg.intensities  # optional
		self.latest_scan = filtered_msg
	
	def run(self):
		rate = rospy.Rate(10)  # 10 Hz
		prev_time = rospy.Time.now()
		
		while not rospy.is_shutdown():
			now = rospy.Time.now()
			dt = (now - prev_time).to_sec()
			prev_time = now
			
			if self.latest_odom is not None:
				v = self.latest_odom.twist.twist.linear.x
				w = self.latest_odom.twist.twist.angular.z
				self.prediction_update(v, w, dt)

			if self.latest_scan is not None:
				landmarks = self.detect_landmarks(self.latest_scan)
				self.batch_measurement_update(landmarks)
				self.latest_scan = None  # Reset scan to avoid reprocessing

			self.publish_landmarks_rviz()
			self.publish_landmark_covariances()
			self.publish_robot_pose()
			rate.sleep()
		
	def detect_landmarks(self, scan):
		angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
		valid = np.logical_and(scan.range_min < np.array(scan.ranges),np.array(scan.ranges) < scan.range_max)
		ranges = np.array(scan.ranges)[valid]
		angles = angles[valid]
		
		# Local coordinates
		x = ranges*np.cos(angles)
		y = ranges*np.sin(angles)
		points = np.vstack((x, y)).T
		
		if len(points) == 0:
			return []
		
		clustering = DBSCAN(eps=0.3, min_samples=5).fit(points)
		labels = clustering.labels_
		landmarks = []
		x, y, theta = self.mu[0:3,0]
		
		for label in set(labels):
			if label == -1:
				continue
			cluster_points = points[labels == label]
			if len(cluster_points) < 5:
				continue
			
			try:
				xc, yc, r, sigma = taubinSVD(cluster_points)
				
				if 0.05 < r < 0.25 and sigma < 0.05:
					xc_global = x + np.cos(theta) * xc - np.sin(theta) * yc
					yc_global = y + np.sin(theta) * xc + np.cos(theta) * yc
					
					landmarks.append((xc_global, yc_global))
			except Exception as e:
				if self.debug:
					rospy.logwarn(f"[Landmark Fit] Circle fit failed: {e}")
				continue
		
		
		return landmarks
			
	def data_associate(self, lx_detected, ly_detected, z):
		best_lm_index = None
		best_z_hat = None
		best_H = None
		min_dist = float('inf')
		min_euclidean_dist = 0.5
		x, y, theta_r = self.mu[0:3,0]
		
		for landmark_index in self.landmark_indices:
			idx = 3+2*landmark_index
			lm = self.mu[idx:idx+2, 0]
			
			dx_euclid = lx_detected - lm[0]
			dy_euclid = ly_detected - lm[1]
			euclid_dist = np.sqrt(dx_euclid ** 2 + dy_euclid ** 2)
			
			if euclid_dist < min_euclidean_dist:
				dx, dy = lm[0] - x, lm[1] - y
				q = dx**2 + dy**2
				r_hat = np.sqrt(q)
				b_hat = np.arctan2(dy, dx) - theta_r
				b_hat = (b_hat + np.pi) % (2 * np.pi) - np.pi
				
				z_hat = np.array([[r_hat], [b_hat]])
				
				H_low = np.array([
					[-dx / r_hat, -dy / r_hat, 0, dx / r_hat, dy / r_hat],
					[dy / q, -dx / q, -1, -dy / q, dx / q]
				])
				
				Fxj = np.zeros((5, self.mu.shape[0]))
				Fxj[0:3, 0:3] = np.eye(3)
				Fxj[3:5, idx:idx + 2] = np.eye(2)
				H = H_low @ Fxj
			
				try:
					S = H @ self.sigma @ H.T + self.R
					dz = z - z_hat
					dz[1] = (dz[1] + np.pi) % (2 * np.pi) - np.pi
					m_dist = dz.T @ np.linalg.inv(S) @ dz
				except ValueError as e:
					if self.debug:
						print("\n[Data Association] Shape mismatch")
						print("H:", H.shape)
						print("sigma:", self.sigma.shape)
						print("R:", self.R.shape)
						print("z:", z.shape)
						print("z_hat:", z_hat.shape)
						print("dz:", dz.shape if 'dz' in locals() else "undefined")
					raise e
			
				if m_dist < min_dist and m_dist < 5.99:
					best_lm_index = landmark_index
					min_dist = m_dist
					best_z_hat = z_hat
					best_H = H
		
		if best_lm_index is None:
			return None
		else:
			return best_lm_index, best_z_hat, best_H
			
			

	def initialize_landmark(self, landmark_xy):
		lx_global, ly_global = landmark_xy
		x, y, theta = self.mu[0:3,0]

		try:
			self.mu = np.vstack([self.mu, [[lx_global], [ly_global]]])
			
			idx = (self.mu.shape[0] - 3) // 2 - 1
			self.landmark_indices.append(idx)
			
			
			new_size = self.sigma.shape[0]+2
			
			Fxj = np.zeros((5, new_size))
			Fxj[0:3, 0:3] = np.eye(3)
			Fxj[3:5, 3+2*idx: 3+2*idx+2] = np.eye(2)
			
			new_sigma = np.zeros((new_size, new_size))
			new_sigma[:self.sigma.shape[0], :self.sigma.shape[1]] = self.sigma
			
			
			R_aug = np.zeros((new_size, new_size))
			R_aug[-2:, -2:] = self.R
			self.sigma = new_sigma + R_aug

		except ValueError as e:
			if self.debug:
				print("\n[Initialize Landmark] Shape mismatch")
				print("Fxj:", Fxj.shape)
				print("R:", self.R.shape)
				print("R_aug:", R_aug.shape if 'R_aug' in locals() else "undefined")
				print("new_sigma:", new_sigma.shape)
				print("sigma:", self.sigma.shape)
			raise e
		
		self.publish_landmarks_rviz()
	
	def prediction_update(self, v, w, dt):
		x, y, theta = self.mu[0:3, 0]
		# Motion update
		if np.abs(w) > 1e-3:
			dx = -(v/w)*np.sin(theta)+(v/w)*np.sin(theta+w*dt)
			dy = (v/w)*np.cos(theta)-(v/w)*np.cos(theta+w*dt)
		else:
			dx = v*np.cos(theta)*dt
			dy = v*np.sin(theta)*dt
		dtheta = w*dt
		
		# Motion
		self.mu[0,0] += dx
		self.mu[1,0] += dy
		self.mu[2,0] = (self.mu[2,0] + dtheta + np.pi)%(2*np.pi) - np.pi

		# Jacobians of the motion model
		Gx = np.eye(3)
		if np.abs(w) > 1e-3:
			Gx[0, 2] = -(v/w)*np.cos(theta)+(v/w)*np.cos(theta+w*dt)
			Gx[1, 2] = -(v/w)*np.sin(theta)+(v/w)*np.sin(theta+w*dt)
		else:
			Gx[0, 2] = -v*np.sin(theta)*dt
			Gx[1, 2] = v*np.cos(theta)*dt
		
		size = self.mu.shape[0]
		G = np.eye(size)
		G[0:3,0:3] = Gx

		# Use of Fx matrix to map robot motion
		Fx = np.zeros((3, size))
		Fx[0:3,0:3] = np.eye(3)		
		
		try:
			self.sigma = G @ self.sigma @ G.T + Fx.T @ self.Q @ Fx
		except ValueError as e:
			if self.debug:
				print("\n[Prediction Update] Matrix shape mismatch:")
				print("G:", G.shape)
				print("sigma:", self.sigma.shape)
				print("G.T:", G.T.shape)
				print("Fx.T:", Fx.T.shape)
				print("Q:", self.Q.shape)
				print("Fx:", Fx.shape)
			raise e

	def batch_measurement_update(self, measurements):
		x, y, theta = self.mu[0:3,0]
		for lx_global, ly_global in measurements:
			dx = lx_global-x
			dy = ly_global-y
			r = np.sqrt(dx ** 2 + dy ** 2)
			b = np.arctan2(dy, dx) - theta
			b = (b + np.pi) % (2 * np.pi) - np.pi
			z = np.array([[r], [b]])
			
			assoc = self.data_associate(lx_global, ly_global, z)
			if assoc is None:
				self.initialize_landmark((lx_global, ly_global))
			else:
				lm_idx, z_hat, H = assoc
				try:
					S = H @ self.sigma @ H.T + self.R
					K = self.sigma @ H.T @ np.linalg.inv(S)
					dz = z - z_hat
					dz[1] = (dz[1] + np.pi) % (2 * np.pi) - np.pi
					self.mu += K @ dz
					self.mu[2, 0] = (self.mu[2, 0] + np.pi) % (2 * np.pi) - np.pi
					self.sigma = (np.eye(self.mu.shape[0])- K @ H) @ self.sigma
				except ValueError as e:
					if self.debug:
						print("\n[Measurement Update] Shape mismatch")
						print("H:", H.shape)
						print("sigma:", self.sigma.shape)
						print("R:", self.R.shape)
						print("z:", z.shape)
						print("z_hat:", z_hat.shape)
						print("K:", K.shape if 'K' in locals() else "undefined")
						print("dz:", dz.shape if 'dz' in locals() else "undefined")
					raise e
		self.publish_landmarks_rviz()
		
		
			
	def publish_landmarks_rviz(self):
		ma = MarkerArray()
		for i,  lm_idx in enumerate(self.landmark_indices):
			idx = 3 + 2 * lm_idx
			lm = self.mu[idx:idx+2, 0]
			
			# Landmark sphere
			marker = Marker()
			marker.header.frame_id = 'odom'
			marker.header.stamp = rospy.Time.now()
			marker.id = i
			marker.type = Marker.SPHERE
			marker.action = Marker.ADD
			marker.pose.position.x = lm[0]
			marker.pose.position.y = lm[1]
			marker.pose.position.z = 0.2
			marker.pose.orientation.x = 0.0
			marker.pose.orientation.y = 0.0
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.scale.x = 0.2
			marker.scale.y = 0.2
			marker.scale.z = 0.2
			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0
			marker.color.a = 1.0
			marker.lifetime = rospy.Duration(0)
			ma.markers.append(marker)
		self.marker_pub.publish(ma)
		
	def publish_markers(self, centroids):
		ma = MarkerArray()
		for i, (x, y) in enumerate(centroids):
			marker = Marker()
			marker.header.frame_id = "odom"
			marker.header.stamp = rospy.Time.now()
			marker.ns = "pillars"
			marker.id = i
			marker.type = Marker.SPHERE
			marker.action = Marker.ADD
			marker.pose.position.x = x
			marker.pose.position.y = y
			marker.pose.position.z = 0.1
			marker.pose.orientation.w = 1.0
			marker.scale.x = 0.15
			marker.scale.y = 0.15
			marker.scale.z = 0.15
			marker.color.r = 1.0
			marker.color.g = 0.5
			marker.color.b = 0.0
			marker.color.a = 1.0
			marker.lifetime = rospy.Duration(0)  # Short lifetime for real-time refresh
			ma.markers.append(marker)
		
		self.pil_pub.publish(ma)
	
	def publish_landmark_covariances(self):
		time_now = rospy.Time.now()
		num_landmarks = (self.mu.shape[0] - 3) // 2
		sigma_sym = 0.5 * (self.sigma + self.sigma.T)
		
		for i in range(num_landmarks):
			idx = 3 + 2 * i
			
			# Create message
			msg = PoseWithCovarianceStamped()
			msg.header.stamp = time_now
			msg.header.frame_id = "odom"
			# Set position
			msg.pose.pose.position.x = self.mu[idx, 0]
			msg.pose.pose.position.y = self.mu[idx + 1, 0]
			msg.pose.pose.position.z = 0.0
			msg.pose.pose.orientation.w = 1.0  # Neutral quaternion
			# Extract 2x2 covariance block
			cov_2x2 = sigma_sym[idx:idx+2, idx:idx+2]
			
			# Optionally clip small negatives due to numerical errors
			eigvals, eigvecs = np.linalg.eigh(cov_2x2)
			eigvals_clipped = np.clip(eigvals, 1e-6, None)
			cov_2x2_psd = eigvecs @ np.diag(eigvals_clipped) @ eigvecs.T
			# Fill into 6x6 covariance matrix
			cov_list = np.zeros((6, 6))
			cov_list[0, 0] = cov_2x2_psd[0, 0]
			cov_list[0, 1] = cov_2x2_psd[0, 1]
			cov_list[1, 0] = cov_2x2_psd[1, 0]
			cov_list[1, 1] = cov_2x2_psd[1, 1]
			msg.pose.covariance = cov_list.flatten().tolist()
			# Publish
			self.landmark_pubs[i].publish(msg)
		
	def publish_robot_pose(self):
		pose_msg = PoseWithCovarianceStamped()
		pose_msg.header.stamp = rospy.Time.now()
		pose_msg.header.frame_id = "odom"
		
		# Robot pose from EKF state vector
		x, y, theta = self.mu[0:3, 0]
		
		pose_msg.pose.pose.position.x = x
		pose_msg.pose.pose.position.y = y
		pose_msg.pose.pose.position.z = 0.0
		q = tft.quaternion_from_euler(0, 0, theta)
		pose_msg.pose.pose.orientation.x = q[0]
		pose_msg.pose.pose.orientation.y = q[1]
		pose_msg.pose.pose.orientation.z = q[2]
		pose_msg.pose.pose.orientation.w = q[3]
		
		# Extract top-left 3x3 block of covariance matrix for robot pose
		cov_matrix = self.sigma[0:3, 0:3]
		cov_list = np.zeros(36)  # Full 6x6 covariance matrix as flat list
		cov_list[0] = cov_matrix[0, 0]  # xx
		cov_list[1] = cov_matrix[0, 1]  # xy
		cov_list[5] = cov_matrix[0, 2]  # xθ
		
		cov_list[6] = cov_matrix[1, 0]  # yx
		cov_list[7] = cov_matrix[1, 1]  # yy
		cov_list[11] = cov_matrix[1, 2]  # yθ
		
		cov_list[30] = cov_matrix[2, 0]  # θx
		cov_list[31] = cov_matrix[2, 1]  # θy
		cov_list[35] = cov_matrix[2, 2]  # θθ
		
		pose_msg.pose.covariance = cov_list.tolist()
		self.pose_pub.publish(pose_msg)



if __name__ == '__main__':
	try:
		ekf_slam = EKFSLAM()
		ekf_slam.run()
	except rospy.ROSInterruptException:
		pass


