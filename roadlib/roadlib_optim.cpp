/*******************************************************
 * Copyright (C) 2024, GREAT Group, Wuhan University
 * 
 * This file is part of RoadLib.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Yuxuan Zhou (yuxuanzhou@whu.edu.cn)
 *******************************************************/
#include "roadlib.h"
#include <ceres/ceres.h>
#include <Eigen/Sparse>

int PointCloud2Curve2D(const vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points, int dim, VectorXd& K)
{
	MatrixXd U = MatrixXd::Zero(points.size(), dim);
	VectorXd Y = VectorXd::Zero(points.size());
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < dim; j++)
			U(i, j) = pow(points[i].y(), j);
		Y(i) = points[i].x();
	}
	K = (U.transpose() * U).inverse() * U.transpose() * Y;
	double res = sqrt(pow((U * K - Y).norm(), 2) / points.size());
	if (res < 5) return 0;
	else return -1;
}

int LineCluster2SingleLine(const PatchType road_class, const vector<shared_ptr<RoadInstancePatch>>& lines_in, shared_ptr<RoadInstancePatch>& line_est, Eigen::Matrix3d Rwv)
{
	vector<RoadInstancePatch> lines;
	for (int i = 0; i < lines_in.size(); i++)
	{
		if (lines_in[i]->line_points_metric.size() < 1) continue;
		lines.push_back(*lines_in[i]);
	}
	if (lines_in.size() == 1)
	{
		line_est = lines_in.front();
		return 0;
	}
	if (lines_in.size() == 0)
		return -1;
	vector<Vector3d> all_pts;
	Eigen::Vector3d vector_temp;
	Eigen::Vector3d all_mean; all_mean.setZero();
	Eigen::Matrix3d all_cov; all_cov.setZero();

	for (auto& line_seg : lines)
	{
		for (int i = 0; i < line_seg.line_points_metric.size(); i++)
		{
			all_pts.push_back(line_seg.line_points_metric[i]);
		}
	}
	//if (merge_mode == 0 && all_pts.size() < 10) return -1;
	for (int j = 0; j < all_pts.size(); j++)
	{
		all_mean += all_pts[j];
	}
	all_mean /= all_pts.size();

	for (int j = 0; j < all_pts.size(); j++)
	{
		vector_temp = all_pts[j] - all_mean;
		all_cov += vector_temp * vector_temp.transpose();
	}
	all_cov /= all_pts.size();

	double eigen_value[3];
	Eigen::Vector3d direction;
	JacobiSVD<Eigen::MatrixXd> svd(all_cov, ComputeThinU | ComputeThinV);
	Matrix3d V = svd.matrixV(), U = svd.matrixU();
	Matrix3d S = U.inverse() * all_cov * V.transpose().inverse();
	eigen_value[0] = sqrt(S(0, 0));
	eigen_value[1] = sqrt(S(1, 1));
	eigen_value[2] = sqrt(S(2, 2));
	direction = U.block(0, 0, 3, 1);
	double yaw = m2att(Rwv).z() * 180 / M_PI+90;
	double direction_angle = atan2(direction(1), direction(0));// round(atan2(direction(1), direction(0)) / (M_PI / 8))* (M_PI / 8); // std::cerr << direction_angle << std::endl;
	if (fabs(yaw - direction_angle * 180 / M_PI) < 90) {}
	else
	{
		direction_angle += M_PI;
	}
	Matrix3d direction_rotation;
	direction_rotation << cos(direction_angle), sin(direction_angle), 0,
		-sin(direction_angle), cos(direction_angle), 0,
		0, 0, 1;

	//std::cerr << direction_rotation << std::endl;
	bool is_ref_set = false;
	Vector3d point_ref;
	for (auto& line_seg : lines)
	{
		for (int i = 0; i < line_seg.line_points_metric.size(); i++)
		{
			if (!is_ref_set)
			{
				point_ref = line_seg.line_points_metric[i];
				is_ref_set = true;
			}
			line_seg.line_points_metric[i] = direction_rotation * (line_seg.line_points_metric[i] - point_ref);
		}
	}

	if (isnan(point_ref.x()))
	{
		std::cerr << "NaN!!" << std::endl;
		return -1;
	}
	double min_x = 1e9;
	double max_x = -1e9;

	for (auto& line_seg : lines)
	{
		for (int i = 0; i < line_seg.line_points_metric.size(); i++)
		{
			//std::cerr << line_seg.line_points_metric[i].x() << " ";
			if (line_seg.line_points_metric[i].x() < min_x) min_x = line_seg.line_points_metric[i].x();
			if (line_seg.line_points_metric[i].x() > max_x) max_x = line_seg.line_points_metric[i].x();
		}
	}

	VectorXd x_est((int)ceil(max_x) - (int)floor(min_x) + 1);
	for (int i = 0; i < x_est.rows(); i++)
	{
		x_est(i) = min_x + i;
	}
	//x_est(0) = min_x;
	x_est(x_est.rows() - 1) = max_x;
	int nq = x_est.rows();
	//std::cerr << x_est << std::endl;

	VectorXd y_est = VectorXd::Zero(nq * 2);
	MatrixXd BtPB = MatrixXd::Zero(nq * 2, nq * 2);
	VectorXd BtPl = MatrixXd::Zero(nq * 2, 1);
	MatrixXd BtPB_obs;
	//std::cerr << "Start iteration." << std::endl;
	for (int i_iter = 0; i_iter < 5; i_iter++)
	{
		SparseMatrix<double> BtPBs(nq * 2, nq * 2);
		SparseMatrix<double> BtPls(nq * 2, 1);
		BtPB = MatrixXd::Zero(nq * 2, nq * 2);
		BtPl = MatrixXd::Zero(nq * 2, 1);

		// Line segments (weighted)
		double x, y, z; double xn, yn, zn; double x0, y0, z0; double x1, y1, z1;
		for (int i = 0; i < lines.size(); i++)
		{
			// Point distance (y-axis direction)
			for (int j = 0; j < lines[i].line_points_metric.size(); j++)
			{
				x = lines[i].line_points_metric[j].x();
				y = lines[i].line_points_metric[j].y();
				z = lines[i].line_points_metric[j].z();
				int seg_index = -1;
				for (int m = 0; m < nq - 1; m++) if (x_est(m) < x + 1e-3 && x_est(m + 1) > x - 1e-3) seg_index = m;

				x0 = x_est(seg_index); y0 = y_est(seg_index); z0 = y_est(seg_index + nq);
				x1 = x_est(seg_index + 1); y1 = y_est(seg_index + 1); z1 = y_est(seg_index + 1 + nq);
				Eigen::VectorXd l = Eigen::VectorXd::Zero(2); Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, nq * 2);

				l(0) = y - (y0 + (y1 - y0) / (x1 - x0) * (x - x0));
				l(1) = z - (z0 + (z1 - z0) / (x1 - x0) * (x - x0));
				B(0, seg_index) = -(1 - (x - x0) / (x1 - x0)); B(0, seg_index + 1) = -(x - x0) / (x1 - x0);
				B(1, seg_index + nq) = -(1 - (x - x0) / (x1 - x0)); B(1, seg_index + 1 + nq) = -(x - x0) / (x1 - x0);

				Eigen::MatrixXd BB0(1,2),BB1(1,2);
				BB0 << -(1 - (x - x0) / (x1 - x0)), -(x - x0) / (x1 - x0);
				BB1 << -(1 - (x - x0) / (x1 - x0)), -(x - x0) / (x1 - x0);

				double point_error = sqrt(lines[i].line_points_uncertainty[j](0, 0) + lines[i].line_points_uncertainty[j](1, 1)) / sqrt(2.0);

				Eigen::MatrixXd P = Eigen::MatrixXd::Identity(2, 2) / pow(point_error, 2);
				//BtPB = BtPB + B.transpose() * P * B; BtPl = BtPl + B.transpose() * P * l;
				BtPB.block(seg_index, seg_index, 2, 2) += BB0.transpose()*BB0 / pow(point_error, 2);
				BtPB.block(seg_index+nq, seg_index + nq, 2, 2) += BB1.transpose()*BB1 / pow(point_error, 2);
				BtPl.segment(seg_index, 2) += BB0.transpose() * l(0) / pow(point_error, 2);
				BtPl.segment(seg_index+nq, 2) += BB1.transpose() * l(1) / pow(point_error, 2);
			}
			BtPB_obs = BtPB;

		}

		// Continuity
		for (int i = 0; i < nq - 2; i++)
		{
			Eigen::VectorXd l = Eigen::VectorXd::Zero(2); Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, nq * 2);
			l(0, 0) = (y_est(i + 1) - y_est(i)) - (y_est(i + 2) - y_est(i + 1));
			l(1, 0) = (y_est(nq + i + 1) - y_est(nq + i)) - (y_est(nq + i + 2) - y_est(nq + i + 1));

			B(0, i) = -1; B(0, i + 1) = 2; B(0, i + 2) = -1;
			B(1, nq + i) = -1; B(1, nq + i + 1) = 2; B(1, nq + i + 2) = -1;

			Eigen::MatrixXd BB0 = Eigen::MatrixXd(1, 3); BB0 << -1, 2, -1;
			Eigen::MatrixXd BB1 = Eigen::MatrixXd(1, 3); BB1 << -1, 2, -1;

			Eigen::MatrixXd P;
			//Eigen::MatrixXd P = Eigen::MatrixXd::Identity(2, 2)/0.04;
			if (road_class == PatchType::SOLID)
				P = Eigen::MatrixXd::Identity(2, 2) / 0.04;
			else if (road_class == PatchType::STOP)
				P = Eigen::MatrixXd::Identity(2, 2) / (0.01 * 0.01);
			//BtPB = BtPB + B.transpose() * P * B; BtPl = BtPl + B.transpose() * P * l;

			BtPB.block(i, i, 3, 3) += BB0.transpose() * P(0,0) * BB0;
			BtPB.block(nq+i, nq+i, 3, 3) += BB1.transpose() *P(1,1) * BB1;
			BtPl.segment(i, 3) += BB0.transpose() * P(0, 0) * l(0);
			BtPl.segment(nq + i, 3) += BB1.transpose() * P(1, 1) * l(1);
		}

		BtPBs = BtPB.sparseView();
		BtPls = BtPl.sparseView();
		SimplicialLDLT< SparseMatrix<double>>solver;
		solver.compute(BtPBs);
		Eigen::VectorXd dy = solver.solve(BtPls);
		
		y_est = y_est - dy;
	}

	SparseMatrix<double> BtPBs_obs = (BtPB_obs + MatrixXd::Identity(nq * 2, nq * 2) * 0.00001).sparseView();
	SimplicialLDLT< SparseMatrix<double>>solver;
	solver.compute(BtPBs_obs);
	SparseMatrix<double> I(BtPB_obs.rows(), BtPB_obs.rows());
	I.setIdentity();

	MatrixXd Cov = solver.solve(I).toDense();

	line_est = make_shared<RoadInstancePatch>();
	line_est->road_class = lines_in[0]->road_class;

	int i_start = 0;
	int i_end = nq - 1;
	for (int i = i_start; i <= i_end; i++)
	{
		if (i >= i_start + (i_end-i_start)/2 && Cov(i, i) > 0.35) break;
		line_est->line_points_metric.push_back(direction_rotation.transpose() * Vector3d(x_est(i), y_est(i), y_est(i + nq)) + point_ref);
		line_est->line_points_uncertainty.push_back((Eigen::Matrix3d() << Cov(i, i), 0, 0, 0, Cov(i, i), 0, 0, 0, 1).finished());
	}
	if (line_est->line_points_metric.size() <= 3)
		return -1;
	for (int iii = 0; iii < line_est->line_points_metric.size(); iii++)
	{
		if (isnan(line_est->line_points_metric[iii].x()))
			return -1;
	}

	line_est->valid_add_to_map = true;
	line_est->line_valid = true;
	line_est->merged = true;
	line_est->mean_metric.setZero();
	vector_temp.setZero();

	for (int j = 0; j < line_est->line_points_metric.size(); j++)
	{
		line_est->mean_metric += line_est->line_points_metric[j];
	}
	line_est->mean_metric /= line_est->line_points_metric.size();

	return 0;

}
