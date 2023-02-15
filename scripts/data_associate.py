#!/home/chenz/anaconda3/envs/surroundCam_cap/bin/python
# -*- coding: utf-8 -*-
# 用于接收检测结果和轨迹的状态，完成数据关联，输出匹配结果对和没有匹配观测/轨迹

import rospy
import numpy as np
import message_filters
from numba import jit
from scipy.optimize import linear_sum_assignment
from dist_metrics import *
#TODO :这里实际上不全是Detection的bbox，还有trk的，后续改一下名称
from track_msgs.msg import Detection_list
from track_msgs.msg import Pairs
from track_msgs.msg import StampArray
from track_msgs.srv import Trk_pred
from track_msgs.srv import Trk_update
from track_msgs.srv import Data_association, Data_associationResponse



match_pub = rospy.Publisher("/matched_pair", Pairs, queue_size=10)
unmatch_trk_pub = rospy.Publisher("/unmatched_trk", StampArray, queue_size=10)
unmatch_det_pub = rospy.Publisher("/unmatched_det", StampArray, queue_size=10)

def compute_affinity(dets, trks, metric, trk_inv_inn_matrices=None):
	# compute affinity matrix

	aff_matrix = np.zeros((len(dets), len(trks)), dtype=np.float32)
	for d, det in enumerate(dets):
		for t, trk in enumerate(trks):

			# choose to use different distance metrics
			if 'iou' in metric:    	  dist_now = iou(det, trk, metric)            
			elif metric == 'm_dis':   dist_now = -m_distance(det, trk, trk_inv_inn_matrices[t])
			elif metric == 'euler':   dist_now = -m_distance(det, trk, None)
			elif metric == 'dist_2d': dist_now = -dist_ground(det, trk)              	
			elif metric == 'dist_3d': dist_now = -dist3d(det, trk)              				
			else: assert False, 'error'
			aff_matrix[d, t] = dist_now

	return aff_matrix

def greedy_matching(cost_matrix):
    # association in the greedy manner
    # refer to https://github.com/eddyhkchiu/mahalanobis_3d_multi_object_tracking/blob/master/main.py

    num_dets, num_trks = cost_matrix.shape[0], cost_matrix.shape[1]

    # sort all costs and then convert to 2D
    distance_1d = cost_matrix.reshape(-1)
    index_1d = np.argsort(distance_1d)
    index_2d = np.stack([index_1d // num_trks, index_1d % num_trks], axis=1)

    # assign matches one by one given the sorting, but first come first serves
    det_matches_to_trk = [-1] * num_dets
    trk_matches_to_det = [-1] * num_trks
    matched_indices = []
    for sort_i in range(index_2d.shape[0]):
        det_id = int(index_2d[sort_i][0])
        trk_id = int(index_2d[sort_i][1])

        # if both id has not been matched yet
        if trk_matches_to_det[trk_id] == -1 and det_matches_to_trk[det_id] == -1:
            trk_matches_to_det[trk_id] = det_id
            det_matches_to_trk[det_id] = trk_id
            matched_indices.append([det_id, trk_id])

    return np.asarray(matched_indices)

def process_dets(self, dets):
		# convert each detection into the class Box3D 
		# inputs: 
		# 	dets - a numpy array of detections in the format [[h,w,l,x,y,z,theta],...]

	dets_new = []
	for det in dets:
		det_tmp = Box3D.array2bbox_raw(det)
		dets_new.append(det_tmp)

	return dets_new

def data_association(dets, trks, metric, threshold, algm='greedy', \
	trk_innovation_matrix=None, hypothesis=1):   
	"""
	Assigns detections to tracked object

	dets:  a list of Box3D object
	trks:  a list of Box3D object

	Returns 3 lists of matches, unmatched_dets and unmatched_trks, and total cost, and affinity matrix
    from https://github.com/xinshuoweng/AB3DMOT.git
	"""

	
	dets = process_dets(dets)
	trks = process_dets(trks)
	# if there is no item in either row/col, skip the association and return all as unmatched
	aff_matrix = np.zeros((len(dets), len(trks)), dtype=np.float32)
	if len(trks) == 0: 
		return np.empty((0, 2), dtype=int), np.arange(len(dets)), [], 0, aff_matrix
	if len(dets) == 0: 
		return np.empty((0, 2), dtype=int), [], np.arange(len(trks)), 0, aff_matrix		
	
	# prepare inverse innovation matrix for m_dis
	if metric == 'm_dis':
		assert trk_innovation_matrix is not None, 'error'
		trk_inv_inn_matrices = [np.linalg.inv(m) for m in trk_innovation_matrix]
	else:
		trk_inv_inn_matrices = None

	# compute affinity matrix
	aff_matrix = compute_affinity(dets, trks, metric, trk_inv_inn_matrices)

	# association based on the affinity matrix
	if hypothesis == 1:
		if algm == 'hungar':
			row_ind, col_ind = linear_sum_assignment(-aff_matrix)      	# hougarian algorithm
			matched_indices = np.stack((row_ind, col_ind), axis=1)
		elif algm == 'greedy':
			matched_indices = greedy_matching(-aff_matrix) 				# greedy matching
		else: assert False, 'error'
	else:
		nouse = True
        # cost_list, hun_list = best_k_matching(-aff_matrix, hypothesis)

	# compute total cost
	cost = 0
	for row_index in range(matched_indices.shape[0]):
		cost -= aff_matrix[matched_indices[row_index, 0], matched_indices[row_index, 1]]

	# save for unmatched objects
	unmatched_dets = []
	for d, det in enumerate(dets):
		if (d not in matched_indices[:, 0]): unmatched_dets.append(d)
	unmatched_trks = []
	for t, trk in enumerate(trks):
		if (t not in matched_indices[:, 1]): unmatched_trks.append(t)

	# filter out matches with low affinity
	matches = []
	for m in matched_indices:
		if (aff_matrix[m[0], m[1]] < threshold):
			unmatched_dets.append(m[0])
			unmatched_trks.append(m[1])
		else: matches.append(m.reshape(1, 2))
	if len(matches) == 0: 
		matches = np.empty((0, 2),dtype=int)
	else: matches = np.concatenate(matches, axis=0)

	return matches, np.array(unmatched_dets), np.array(unmatched_trks), cost, aff_matrix

def associate_Callback(dets):
	
	rospy.loginfo("Data association Into callback")

	unpack_dets = []
	for det in dets.detecs:
		unpack_dets.append([det.siz.x, det.siz.y, det.siz.z, det.pos.x, det.pos.y, det.pos.z, det.alp].reshape(1,7))
	try:
		get_trk_preds = rospy.ServiceProxy('/trk_predict', Trk_pred)
		trks = get_trk_preds(dets.header.stamp.secs / 10)
	except rospy.ServiceException as e:
		rospy.logwarn(e)
	
	unpack_trks = []
	for trk in trks.detecs:
		unpack_trks.append([trk.siz.x, trk.siz.y, trk.siz.z, trk.pos.x, trk.pos.y, trk.pos.z, trk.alp].reshape(1,7))
	


	matches,unmatch_dets,unmatch_trks, cost, aff_matrix = data_association(unpack_dets, unpack_trks, "giou_3d", -0.2, algm='hungar')


	pub_match = Pairs()
	pub_match.header = dets.header
	pub_match.dets = matches[:, 0]
	pub_match.trks = matches[:, 1]

	pub_undets = StampArray()
	pub_undets.header = dets.header
	pub_undets.ids = unmatch_dets

	pub_untrks = StampArray()
	pub_untrks.header = dets.header
	pub_untrks.ids = unmatch_trks

	try:
		process_trk_update = rospy.ServiceProxy('/trk_update', Trk_update)
		success = process_trk_update(pub_match, pub_undets, pub_untrks, dets)
	except rospy.ServiceException as e:
		rospy.logwarn(e)
	# match_pub.publish(pub_match)
	# unmatch_det_pub.publish(pub_undets)
	# unmatch_trk_pub.publish(pub_untrks)
	
	if(int(dets.header.stamp.secs) % 1 == 0):
		state = 'Fail!'
		if success:
			state = 'Success!'
		rospy.loginfo("Data association of %d frame finished with %d matches, %d unmatched dets, %d unmatched trks, and updation of trks %s ", 
		int(dets.header.stamp.secs), pub_match.dets.size(), pub_undets.ids.size(), pub_untrks.ids.size(), state)


def srv_associate_Callback(req):
	
	rospy.loginfo("Data association Into callback")

	dets = req.dets
	unpack_dets = []
	for det in dets.detecs:
		unpack_dets.append([det.siz.x, det.siz.y, det.siz.z, det.pos.x, det.pos.y, det.pos.z, det.alp].reshape(1,7))
	
	try:
		get_trk_preds = rospy.ServiceProxy('/trk_predict', Trk_pred)
		pred_res = get_trk_preds(dets.header.stamp.secs / 10)
	except rospy.ServiceException as e:
		rospy.logwarn(e)
	trks = pred_res.trk_predicts
	unpack_trks = []
	for trk in trks.detecs:
		unpack_trks.append([trk.siz.x, trk.siz.y, trk.siz.z, trk.pos.x, trk.pos.y, trk.pos.z, trk.alp].reshape(1,7))
	


	matches,unmatch_dets,unmatch_trks, cost, aff_matrix = data_association(unpack_dets, unpack_trks, "giou_3d", -0.2, algm='hungar')


	pub_match = Pairs()
	pub_match.header = dets.header
	pub_match.dets = matches[:, 0]
	pub_match.trks = matches[:, 1]

	pub_undets = StampArray()
	pub_undets.header = dets.header
	pub_undets.ids = unmatch_dets

	pub_untrks = StampArray()
	pub_untrks.header = dets.header
	pub_untrks.ids = unmatch_trks

	try:
		process_trk_update = rospy.ServiceProxy('/trk_update', Trk_update)
		update_res = process_trk_update(pub_match, pub_undets, pub_untrks, dets)
	except rospy.ServiceException as e:
		rospy.logwarn(e)
	# match_pub.publish(pub_match)
	# unmatch_det_pub.publish(pub_undets)
	# unmatch_trk_pub.publish(pub_untrks)
	
	if(int(dets.header.stamp.secs) % 1 == 0):
		state = 'Fail!'
		if update_res.success:
			state = 'Success!'
		rospy.loginfo("Data association of %d frame finished with %d matches, %d unmatched dets, %d unmatched trks, and updation of trks %s ", 
		int(dets.header.stamp.secs), pub_match.dets.size(), pub_undets.ids.size(), pub_untrks.ids.size(), state)
	
	res = Data_associationResponse()
	res.success =True
	return res

def main():
    
    rospy.init_node('data_association_node', anonymous=True)

    rospy.wait_for_service('/trk_predict')
    rospy.wait_for_service('/trk_update')

    s = rospy.Service('/data_association', Data_association, srv_associate_Callback)
	
    # rospy.Subscriber("/detections", Detection_list, associate_Callback)

    rospy.loginfo("Data association of trk_predict and dets")


    rospy.spin()
if __name__ == '__main__':
	main()
