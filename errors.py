# -*- coding: utf-8 -*-
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
import os
import sys
import itertools
import numpy as np
matplotlib.use("pgf")
pgf_config = {
    "font.family":'serif',
    "font.size": 7.5,
    "pgf.rcfonts": False,
    "text.usetex": True,
    "pgf.preamble": [
        r"\usepackage{unicode-math}",
        r"\setmainfont{Times New Roman}",
        r"\usepackage{xeCJK}",
        r"\setCJKmainfont{SimSun}",
    ],
}
matplotlib.rcParams.update({'font.size': 12})

DPI = 5000

###########################################################################################

input_dir = "/home/fxyttql/data/log/errors"
print("Reading from", input_dir)

## -- type of trajectory and experiment number
if len(sys.argv) == 3:
	traj_types = [sys.argv[1]]
	test_nums = [sys.argv[2]]
else:
	traj_types = ["l"]  # c for circular ---- l for linear
	test_nums = [str(x) for x in range(1)]

###########################################################################################

for traj_type, test_num in itertools.product(traj_types, test_nums):

	print("traj_type:", traj_type, "- test_num: ", test_num)

	## save dir
	save_dir = "/home/fxyttql/data/log/errors/{}{}".format(traj_type, test_num)
	if not os.path.exists(save_dir):
		os.makedirs(save_dir)

	## read files
	errors_pred = pd.read_csv(input_dir + "/errors_pred.csv")
	errors_no_pred = pd.read_csv(input_dir + "/errors_NO_pred.csv")

	# extract data from DataFrame
	t_no_pred = errors_no_pred['t'].tolist()
	ex_no_pred_real = errors_no_pred['ex_real'].tolist()
	ey_no_pred_real = errors_no_pred['ey_real'].tolist()
	ez_no_pred_real = errors_no_pred['ez_real'].tolist()
	ex_no_pred_target = errors_no_pred['ex_target'].tolist()
	ey_no_pred_target = errors_no_pred['ey_target'].tolist()
	ez_no_pred_target = errors_no_pred['ez_target'].tolist()

	t_pred = errors_pred['t'].tolist()
	ex_pred_real = errors_pred['ex_real'].tolist()
	ey_pred_real = errors_pred['ey_real'].tolist()
	ez_pred_real = errors_pred['ez_real'].tolist()
	ex_pred_target = errors_pred['ex_target'].tolist()
	ey_pred_target = errors_pred['ey_target'].tolist()
	ez_pred_target = errors_pred['ez_target'].tolist()

	## error x
	fig, ax = plt.subplots()
	ax.plot(t_no_pred, ex_no_pred_real, 'r', label='无补偿法')
	ax.plot(t_pred, ex_pred_real, 'b', label='有补偿法')
	ax.set(xlabel='时间 (s)', ylabel='x轴误差 (m)')
	plt.axhline(y=0.1, ls="--", linewidth=1, c="green")
	plt.axhline(y=-0.1, ls="--", linewidth=1, c="green")
	# plt.title(' ')
	# plt.annotate('aaa', xy=(2,0.5),xytext=(3,1),arrowprops=dict(width=0.8,facecolor='black',shrink=0.01))
	bottom, top = plt.ylim()  # return the current ylim
	plt.ylim((-1.5, 1.5))   # set the ylim to bottom, top
	ax.grid()
	ax.legend()
	fig.savefig(os.path.join(save_dir, "ex.svg"), format='svg', dpi=5200)
	plt.close()

	## error y
	fig, ax = plt.subplots()
	ax.plot(t_no_pred, ey_no_pred_real, 'r', label='无补偿法')
	ax.plot(t_pred, ey_pred_real, 'b', label='有补偿法')
	ax.set(xlabel='时间 (s)', ylabel='y轴误差 (m)')
	plt.axhline(y=0.1, ls="--", linewidth=1, c="green")
	plt.axhline(y=-0.1, ls="--", linewidth=1, c="green")
	# plt.title(' ')
	# plt.annotate('aaa', xy=(2,0.5),xytext=(3,1),arrowprops=dict(width=0.8,facecolor='black',shrink=0.01))
	plt.ylim((-1.5, 1.5))   # set the ylim to bottom, top
	ax.grid()
	ax.legend()
	fig.savefig(os.path.join(save_dir, "ey.svg"), format='svg', dpi=5200)
	plt.close()

	# error z
	fig, ax = plt.subplots()
	ax.plot(t_no_pred, ez_no_pred_real, 'r', label='无补偿法')
	ax.plot(t_pred, ez_pred_real, 'b', label='有补偿法')
	ax.set(xlabel='时间 (s)', ylabel='error in z (m)')
	ax.grid()
	ax.legend()
	fig.savefig(os.path.join(save_dir, "ez.svg"), format='svg', dpi=5200)
	plt.close()



