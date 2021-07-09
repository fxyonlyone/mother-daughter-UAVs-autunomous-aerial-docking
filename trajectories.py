import matplotlib
import matplotlib.pyplot as plt 
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os
import matplotlib.pyplot as plt
import sys
import itertools

matplotlib.rcParams.update({'font.size': 12})

matplotlib.use("pgf")
pgf_config = {
    "font.family":'serif',
    "font.size": 12,
    "pgf.rcfonts": False,
    "text.usetex": True,
    "pgf.preamble": [
        r"\usepackage{unicode-math}",
        r"\setmainfont{Times New Roman}",
        r"\usepackage{xeCJK}",
        r"\setCJKmainfont{SimSun}",
    ],
}

DPI = 5000

input_dir = "/home/fxyttql/data/log/trajectories"
print("Reading from", input_dir) 


## -- type of trajectory and experiment number
if len(sys.argv) == 3:
	traj_types = [sys.argv[1]]
	test_nums = [sys.argv[2]]
else:
	traj_types = ["l"] # c for circular ---- l for linear
	test_nums = [str(x) for x in range(1)]


for traj_type, test_num in itertools.product(traj_types, test_nums):

	print("traj_type:", traj_type, "- test_num: ", test_num)

	## -- save dir
	save_dir = "/home/fxyttql/data/log/trajectories/{}{}".format(traj_type, test_num)
	if not os.path.exists(save_dir):
		os.makedirs(save_dir)

	##########################################################
	################### w/ PREDICTION ########################
	##########################################################
	trajectories_pred = pd.read_csv(input_dir + "/trajectories_pred.csv")
	# extract data from DataFrame
	ziji_x = trajectories_pred['aX'].tolist()
	ziji_y = trajectories_pred['aY'].tolist()
	ziji_z = trajectories_pred['aZ'].tolist()
	muji_x = trajectories_pred['sX'].tolist()
	muji_y = trajectories_pred['sY'].tolist()
	muji_z = trajectories_pred['sZ'].tolist()

	# 3D
	fig = plt.figure()
	
	ax = fig.gca(projection='3d')
	ax.scatter(ziji_x[0], ziji_y[0], ziji_z[0], c='g',s=12)
	ax.scatter(ziji_x[-1], ziji_y[-1], ziji_z[-1], c='y',s=12)
	ax.scatter(muji_x[0], muji_y[0], muji_z[0], c='g',s=12, marker='s')
	ax.scatter(muji_x[-1], muji_y[-1], muji_z[-1], c='y',s=12, marker='s')
	ax.plot(ziji_x, ziji_y, ziji_z, 'b', label='子机')
	ax.plot(muji_x, muji_y, muji_z, 'r', label='母机')
	ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')
	plt.legend(loc='upper right')
	ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
	ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
	ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
	ax.view_init(elev=15, azim=-45)

	bottom, top = plt.ylim()  # return the current ylim
	plt.ylim((bottom-0.5, top+0.5))   # set the ylim to bottom, top
	ax.legend()
	# fig.savefig(os.path.join(save_dir, "traj3D_pred.pdf"), format='pdf', dpi=DPI)
	fig.savefig(os.path.join(save_dir, "traj3D_pred.svg"), format='svg',dpi=5200)
	# plt.close()

	# 2D
	fig, ax = plt.subplots()
	ax.plot(ziji_x, ziji_y, 'b', label='子机')
	ax.plot(muji_x, muji_y, 'r', label='母机')
	ax.set(xlabel='x (m)', ylabel='y (m)')
	ax.scatter(ziji_x[0], ziji_y[0],  c='g',s=12)
	ax.scatter(ziji_x[-1], ziji_y[-1],  c='y',s=12)
	ax.scatter(muji_x[0], muji_y[0],  c='g',s=12, marker='s')
	ax.scatter(muji_x[-1], muji_y[-1], c='y',s=12, marker='s')
	plt.legend(loc='upper right')
	# if traj_type == "l":
	# 	plt.ylim((-0.5, 0.5))
	# else:
	# 	bottom, top = plt.ylim()  # return the current ylim
	# 	plt.ylim((bottom-1, top+1))   # set the ylim to bottom, top
	ax.legend()
	ax.grid()
	# fig.savefig(os.path.join(save_dir, "traj2D_pred.pdf"), format='pdf', dpi=DPI)
	fig.savefig(os.path.join(save_dir, "traj2D_pred.svg"), format='svg',dpi=5200)
	plt.close()

	# plt.show()
	# exit()

	#########################################################
	####################no_pred #####################
	#########################################################
	trajectories_no_pred = pd.read_csv(input_dir + "/trajectories_NO_pred.csv")

	# extract data from DataFrame
	ziji_x = trajectories_no_pred['aX'].tolist()
	ziji_y = trajectories_no_pred['aY'].tolist()
	ziji_z = trajectories_no_pred['aZ'].tolist()
	muji_x = trajectories_no_pred['sX'].tolist()
	muji_y = trajectories_no_pred['sY'].tolist()
	muji_z = trajectories_no_pred['sZ'].tolist()

	# 3D
	fig = plt.figure()
	ax = fig.gca(projection='3d')
	ax.scatter(ziji_x[0], ziji_y[0], ziji_z[0], c='g',s=12)
	ax.scatter(ziji_x[-1], ziji_y[-1], ziji_z[-1], c='y',s=12)
	ax.scatter(muji_x[0], muji_y[0], muji_z[0], c='g',s=12, marker='s')
	ax.scatter(muji_x[-1], muji_y[-1], muji_z[-1], c='y',s=12, marker='s')
	ax.plot(ziji_x, ziji_y, ziji_z, 'b', label='子机')
	ax.plot(muji_x, muji_y, muji_z, 'r', label='母机')
	plt.legend(loc='upper right')
	ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')
	
	ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
	ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
	ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
	ax.view_init(elev=15, azim=-45)
	ax.legend()
	bottom, top = plt.ylim()  # return the current ylim
	plt.ylim((bottom-0.5, top+0.5))   # set the ylim to bottom, top
	# fig.savefig(os.path.join(save_dir, "traj3D_NO_pred.pdf"), format='pdf', dpi=DPI)
	fig.savefig(os.path.join(save_dir, "traj3D_NO_pred.svg"), format='svg',dpi=5200)
	plt.close()
	
	# 2D
	fig, ax = plt.subplots()
	ax.plot(ziji_x, ziji_y, 'b', label='子机')
	ax.plot(muji_x, muji_y, 'r', label='母机')
	plt.legend(( 'Masked if > 0.5', 'Masked if < -0.5'), loc='upper right')  
	ax.set(xlabel='x (m)', ylabel='y (m)')
	ax.scatter(ziji_x[0], ziji_y[0],  c='g',s=12)
	ax.scatter(ziji_x[-1], ziji_y[-1],  c='y',s=12)
	ax.scatter(muji_x[0], muji_y[0],c='g',s=12, marker='s')
	ax.scatter(muji_x[-1], muji_y[-1],c='y',s=12, marker='s')
	plt.legend(loc='upper right')
	# if traj_type == "l":
	# 	plt.ylim((-0.5, 0.5))
	# else:
	# 	bottom, top = plt.ylim()  # return the current ylim
	# 	plt.ylim((bottom-1, top+1))   # set the ylim to bottom, top
	ax.legend()
	ax.grid()
	# fig.savefig(os.path.join(save_dir, "traj2D_NO_pred.pdf"), format='pdf', dpi=DPI)
	fig.savefig(os.path.join(save_dir, "traj2D_NO_pred.svg"), format='svg',dpi=5200)
	plt.close()
