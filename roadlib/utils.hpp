#pragma once
#include <Eigen/Core>
#include <fstream>
#define G_PI        3.14159265358979311599796346854419e0    // pi
#define SQR(x)   ((x)*(x))
#define SQRT(x)  ((x)<=0.0?0.0:sqrt(x))
#define Aell        6378137.000           // semi major axes [m]
#define Finv        298.257223563         // inverse flattening [-]
#define D2R         (G_PI/180.0)          // deg to rad
#define R2D         (180.0/G_PI)          // rad to deg
using namespace Eigen;
using namespace std;

inline bool double_eq(const double& a, const double& b)
{
	//  if ( fabs(a-b)< 1e-30) return true;   // must be editeed with machine epsilon (FOLLOWING FLOAT BELLOW!)
	if (fabs(a - b) < std::numeric_limits<double>::epsilon()) return true;   // must be editeed with machine epsilon (FOLLOWING FLOAT BELLOW!)
	else return false;
}

inline Matrix3d askew(const Vector3d& v)
{
	Matrix3d vnx;
	vnx << 0, -v(2), v(1),
		v(2), 0, -v(0),
		-v(1), v(0), 0;
	return vnx;
}

inline Matrix3d rv2m(const Vector3d& rv)
{
	double n2 = rv.norm(), a, b, n;
	if (n2 < (M_PI / 180.0*M_PI / 180.0))
	{
		a = 1 - n2 * (1 / 6 - n2 / 120);
		b = 0.5 - n2 * (1 / 24 - n2 / 720);
	}
	else
	{
		n = sqrt(n2);
		a = sin(n) / n;
		b = (1 - cos(n)) / n2;
	}
	Matrix3d rx = askew(rv);
	Matrix3d DCM = Matrix3d::Identity() + a * rx + b * rx*rx;
	return DCM;

}

inline Vector3d m2rv(const Matrix3d& Cnb)
{
	double q0, q1, q2, q3, qq4;
	if (Cnb(0, 0) >= Cnb(1, 1) + Cnb(2, 2))
	{
		q1 = 0.5*sqrt(1 + Cnb(0, 0) - Cnb(1, 1) - Cnb(2, 2));  qq4 = 4 * q1;
		q0 = (Cnb(2, 1) - Cnb(1, 2)) / qq4; q2 = (Cnb(0, 1) + Cnb(1, 0)) / qq4; q3 = (Cnb(0, 2) + Cnb(2, 0)) / qq4;
	}
	else if (Cnb(1, 1) >= Cnb(0, 0) + Cnb(2, 2))
	{
		q2 = 0.5*sqrt(1 - Cnb(0, 0) + Cnb(1, 1) - Cnb(2, 2));  qq4 = 4 * q2;
		q0 = (Cnb(0, 2) - Cnb(2, 0)) / qq4; q1 = (Cnb(0, 1) + Cnb(1, 0)) / qq4; q3 = (Cnb(1, 2) + Cnb(2, 1)) / qq4;
	}
	else if (Cnb(2, 2) >= Cnb(0, 0) + Cnb(1, 1))
	{
		q3 = 0.5*sqrt(1 - Cnb(0, 0) - Cnb(1, 1) + Cnb(2, 2));  qq4 = 4 * q3;
		q0 = (Cnb(1, 0) - Cnb(0, 1)) / qq4; q1 = (Cnb(0, 2) + Cnb(2, 0)) / qq4; q2 = (Cnb(1, 2) + Cnb(2, 1)) / qq4;
	}
	else
	{
		q0 = 0.5*sqrt(1 + Cnb(0, 0) + Cnb(1, 1) + Cnb(2, 2));  qq4 = 4 * q0;
		q1 = (Cnb(2, 1) - Cnb(1, 2)) / qq4; q2 = (Cnb(0, 2) - Cnb(2, 0)) / qq4; q3 = (Cnb(1, 0) - Cnb(0, 1)) / qq4;
	}
	double nq = sqrt(q0*q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= nq; q1 /= nq; q2 /= nq; q3 /= nq;

	if (q0 < 0) { q0 = -q0, q1 = -q1, q2 = -q2, q3 = -q3; }
	if (q0 > 1.0) q0 = 1.0;
	double n2 = acos(q0), f;
	if (n2 > 1.0e-20)
	{
		f = 2.0 / (sin(n2) / n2);
	}
	else
	{
		f = 2.0;
	}
	return Vector3d(q1, q2, q3)*f;
}

inline Matrix3d a2mat(const Vector3d& att)
{
	//double sy = sin(att(0)), cy = cos(att(0));
	//double sp = sin(att(1)), cp = cos(att(1));
	//double sr = sin(att(2)), cr = cos(att(2));

	double sp = sin(att(0)), cp = cos(att(0));
	double sr = sin(att(1)), cr = cos(att(1));
	double sy = sin(att(2)), cy = cos(att(2));

	Matrix3d m;
	m << cy * cr - sy * sp*sr, -sy * cp, cy*sr + sy * sp*cr,
		sy*cr + cy * sp*sr, cy*cp, sy*sr - cy * sp*cr,
		-cp * sr, sp, cp*cr;
	return m;
}

inline Vector3d m2att(const Matrix3d& m)
{
	Vector3d att;
	//att(0) = atan2(-m(0, 1), m(1, 1));
	//att(1) = asin(m(2, 1));
	//att(2) = atan2(-m(2, 0), m(2, 2));

	att(0) = asin(m(2, 1));
	att(1) = atan2(-m(2, 0), m(2, 2));
	att(2) = atan2(-m(0, 1), m(1, 1));
	//if (att(2) > 0 && att(2) < 180)att(2) = 2 * glv.PI - att(2);
	//if (att(2) > -180 && att(2) < 0)att(2) = -att(2);
	return att;
}

// Ellipsoidal to XYZ coordinates
// ----------
inline int ell2xyz(const double* Ell, double* XYZ, bool degrees)
{
	const double bell = Aell * (1.0 - 1.0 / Finv);
	const double e2 = (Aell * Aell - bell * bell) / (Aell * Aell);

	double nn;
	double hsl = Ell[2]; // [m] above mean sea level

	if (degrees) {
		nn = Aell / sqrt(1.0 - e2 * sin(Ell[0] * D2R) * sin(Ell[0] * D2R));
		XYZ[0] = (nn + hsl) * cos(Ell[0] * D2R) * cos(Ell[1] * D2R);
		XYZ[1] = (nn + hsl) * cos(Ell[0] * D2R) * sin(Ell[1] * D2R);
		XYZ[2] = ((1 - e2) * nn + hsl) * sin(Ell[0] * D2R);
	}
	else {
		nn = Aell / sqrt(1.0 - e2 * sin(Ell[0]) * sin(Ell[0]));
		XYZ[0] = (nn + hsl) * cos(Ell[0]) * cos(Ell[1]);
		XYZ[1] = (nn + hsl) * cos(Ell[0]) * sin(Ell[1]);
		XYZ[2] = ((1 - e2) * nn + hsl) * sin(Ell[0]);
	}

	return 1;
}


// XYZ to Ellipsoidal coordinates
// ----------
inline int xyz2ell(const double* XYZ, double* Ell, bool degrees)
{
	const double bell = Aell * (1.0 - 1.0 / Finv);
	const double e2 = (Aell * Aell - bell * bell) / (Aell * Aell);
	const double e2c = (Aell * Aell - bell * bell) / (bell * bell);

	double nn, ss, zps, hOld, phiOld, theta, sin3, cos3;

	ss = sqrt(XYZ[0] * XYZ[0] + XYZ[1] * XYZ[1]);

	if (double_eq(ss, 0.0)) { Ell[0] = -999; Ell[1] = -999; Ell[2] = -999; return 1; }

	zps = XYZ[2] / ss;
	theta = atan((XYZ[2] * Aell) / (ss * bell));
	sin3 = sin(theta) * sin(theta) * sin(theta);
	cos3 = cos(theta) * cos(theta) * cos(theta);

	// Closed formula
	Ell[0] = atan((XYZ[2] + e2c * bell * sin3) / (ss - e2 * Aell * cos3));
	Ell[1] = atan2(XYZ[1], XYZ[0]);
	nn = Aell / sqrt(1.0 - e2 * sin(Ell[0]) * sin(Ell[0]));
	Ell[2] = ss / cos(Ell[0]) - nn;

	const int MAXITER = 100;
	for (int ii = 1; ii <= MAXITER; ii++) {
		nn = Aell / sqrt(1.0 - e2 * sin(Ell[0]) * sin(Ell[0]));
		hOld = Ell[2];
		phiOld = Ell[0];
		Ell[2] = ss / cos(Ell[0]) - nn;
		Ell[0] = atan(zps / (1.0 - e2 * nn / (nn + Ell[2])));

		if (fabs(phiOld - Ell[0]) <= 1.0e-11 && fabs(hOld - Ell[2]) <= 1.0e-5) {

			// always convert longitude to 0-360
			if (Ell[1] < 0.0) Ell[1] += 2 * G_PI;

			if (degrees) { Ell[0] *= R2D; Ell[1] *= R2D; }

			return 0;
		}
	}

	return 1;
}


// XYZ to Topocentric/Local (North, East, Up) coordinates
// ----------
inline int xyz2neu(const double* XYZ, const double* XYZ_Ref, double* neu)
{
	double ele[3];
	xyz2ell(XYZ_Ref, ele, false);

	double r[3] = { 0.0 };
	r[0] = XYZ[0] - XYZ_Ref[0];
	r[1] = XYZ[1] - XYZ_Ref[1];
	r[2] = XYZ[2] - XYZ_Ref[2];

	double sinPhi = sin(ele[0]);
	double cosPhi = cos(ele[0]);
	double sinLam = sin(ele[1]);
	double cosLam = cos(ele[1]);

	neu[0] = -sinPhi * cosLam * r[0]
		- sinPhi * sinLam * r[1]
		+ cosPhi * r[2];

	neu[1] = -sinLam * r[0]
		+ cosLam * r[1];

	neu[2] = +cosPhi * cosLam * r[0]
		+ cosPhi * sinLam * r[1]
		+ sinPhi * r[2];

	return 1;
}

inline Eigen::Matrix3d calcRne(Eigen::Vector3d xyz)
{
	double XYZ_Ref[3] = { xyz(0),xyz(1),xyz(2) };
	double ell[3];
	xyz2ell(XYZ_Ref, ell, false);
	double sinPhi = sin(ell[0]);
	double cosPhi = cos(ell[0]);
	double sinLam = sin(ell[1]);
	double cosLam = cos(ell[1]);
	Eigen::Matrix3d Rne;
	Rne << -sinLam, cosLam, 0,
		-sinPhi * cosLam, -sinPhi * sinLam, cosPhi,
		cosPhi * cosLam, cosPhi * sinLam, sinPhi;

	return Rne;
}

struct Tpoint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
	Eigen::Vector3d v;
	bool valid;
};

struct Trajectory
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
	Eigen::Vector3d ref;
	map<double, Tpoint> poses;
};

//struct TrajectoryPoint
//{
//	double time;
//	Vector3d pos;
//	Vector3d att;
//	Vector3d vel;
//	bool valid;
//
//	pair<Matrix3d, Vector3d> RefTo(TrajectoryPoint p)
//	{
//		Matrix3d Rne = calcRne(p.pos);
//		Matrix3d Rn1e = calcRne(this->pos);
//		Matrix3d Rn1b = a2mat(this->att);
//		Vector3d ten = p.pos;
//		Vector3d teb = this->pos;
//		Vector3d tnb = Rne * (teb - ten);
//		Matrix3d Rnb = Rne * Rn1e.transpose() * Rn1b;
//		return make_pair(Rnb, tnb);
//
//	}
//};

struct t_graphpose
{
	double time;
	Matrix3d R;
	Vector3d t;
	double* t_array;
	double* q_array;
	Vector3d pos_orig;
	Vector3d att_orig;
};
//
//inline map<double, TrajectoryPoint> load_global_traj(const string& filename)
//{
//
//}
//
//inline map<double, TrajectoryPoint> load_local_traj(const string& filename, const Vector3d& xyz_ref)
//{
//	map<double, TrajectoryPoint> traj;
//	string line;
//	stringstream ss;
//	ifstream ifs(filename);
//
//	Matrix3d Rne = calcRne(xyz_ref);
//
//	double sow, x, y, z, vx, vy, vz, pitch, roll, yaw;
//	while (ifs.good())
//	{
//		getline(ifs, line);
//		if (line.size() < 1) continue;
//		if (line[0] == '#') continue;
//		ss.clear(); ss.str("");
//		ss << line;
//		ss >> sow >> x >> y >> z >> vx >> vy >> vz >> pitch >> roll >> yaw;
//		bool valid = true;
//		if (line.find("Float") != string::npos) valid = false;
//		Vector3d xyz = xyz_ref + Rne.transpose() * Vector3d(x, y, z);
//		Vector3d vxyz = Rne.transpose() * Vector3d(vx, vy, vz);
//
//		traj[sow] = { sow,xyz,Vector3d(pitch,roll,yaw) * M_PI / 180,vxyz,valid };
//	}
//	return traj;
//}

inline Trajectory load_vio(const string& filename, const Vector3d& xyz_ref)
{
	Trajectory traj;
	string line;
	stringstream ss;
	ifstream ifs(filename);

	traj.ref = xyz_ref;

	double sow, x, y, z, vx, vy, vz, pitch, roll, yaw;
	while (ifs.good())
	{
		getline(ifs, line);
		if (line.size() < 1) continue;
		if (line[0] == '#') continue;
		ss.clear(); ss.str("");
		ss << line;
		ss >> sow >> x >> y >> z >> vx >> vy >> vz >> pitch >> roll >> yaw;
		bool valid = true;
		traj.poses[sow] = { a2mat(Vector3d(pitch,roll,yaw) * M_PI / 180),Vector3d(x, y, z) ,Vector3d(vx, vy, vz), valid };
	}
	return traj;
}

inline Trajectory load_trajectory(const string & filename)
{
	Trajectory traj;
	string line;
	stringstream ss;
	ifstream ifs(filename);

	double sow, x, y, z, vx, vy, vz, pitch, roll, yaw;
	double gx, gy, gz, ax, ay, az;
	string meas;
	int nsat;
	double pdop;
	string amb;
	double ratio;
	bool is_ref_set = false;
	Matrix3d Rn0e;
	while (ifs.good())
	{
		getline(ifs, line);
		if (line.size() < 1) continue;
		if (line[0] == '#') continue;
		ss.clear(); ss.str("");
		ss << line;
		ss >> sow >> x >> y >> z >> vx >> vy >> vz >> pitch >> roll >> yaw;
		ss >> gx >> gy >> gz >> ax >> ay >> az >> meas >> nsat >> pdop >> amb >> ratio;
		bool valid = true;
		if (line.find("Float") != string::npos || nsat<12) valid = false;
		Eigen::Vector3d xyz = Vector3d(x, y, z);
		Eigen::Vector3d vxyz = Vector3d(vx, vy, vz);
		Eigen::Matrix3d Rnb = a2mat(Vector3d(pitch, roll, -yaw) * M_PI / 180);

		if (!is_ref_set)
		{
			traj.ref = Vector3d(x, y, z);
			Rn0e = calcRne(traj.ref);
			is_ref_set = true;
		}
		Eigen::Vector3d twb = Rn0e * (xyz - traj.ref);
		Eigen::Vector3d vwb = Rn0e * vxyz;
		Eigen::Matrix3d Rn0b = Rn0e * calcRne(xyz).transpose() * Rnb;

		traj.poses[sow] = { Rn0b,twb,vwb, valid };
	}

	return traj;
}
inline Trajectory load_gnss(const string& filename)
{
	Trajectory traj;
	string line;
	stringstream ss;
	ifstream ifs(filename);

	double sow, x, y, z, vx, vy, vz, xrms, yrms, zrms, vxrms, vyrms, vzrms;
	int nsat;
	double pdop;
	double sigma0;
	string amb;
	double ratio;
	double bl;
	double qual;
	bool is_ref_set = false;
	Matrix3d Rn0e;

	while (ifs.good())
	{
		getline(ifs, line);
		if (line.size() < 1) continue;
		if (line[0] == '#') continue;
		ss.clear(); ss.str("");
		ss << line;
		ss >> sow >> x >> y >> z >> vx >> vy >> vz;
		ss >> xrms >> yrms >> zrms >> vxrms >> vyrms >> vzrms;
		ss >> nsat >> pdop >> sigma0 >> amb >> ratio >> bl >> qual;
		bool valid = true;
		if (line.find("Float") != string::npos || nsat < 12) valid = false;
		Eigen::Vector3d xyz = Vector3d(x, y, z);
		Eigen::Vector3d vxyz = Vector3d(vx, vy, vz);

		if (!is_ref_set)
		{
			traj.ref = Vector3d(x, y, z);
			Rn0e = calcRne(traj.ref);
			is_ref_set = true;
		}
		Eigen::Vector3d twb = Rn0e * (xyz - traj.ref);
		Eigen::Vector3d vwb = Rn0e * vxyz;

		traj.poses[sow] = { Eigen::Matrix3d::Identity(),twb,vwb, valid };
	}

	return traj;
}

inline Trajectory load_ie(const string & filename)
{
	Trajectory traj;
	string line;
	stringstream ss;
	ifstream ifs(filename);

	double week, sow, x, y, z, vx, vy, vz, pitch, roll, yaw, temp;
	bool is_ref_set = false;
	Matrix3d Rn0e;
	while (ifs.good())
	{
		getline(ifs, line);
		if (line.size() < 1) continue;
		if (line[0] == '#') continue;
		ss.clear(); ss.str("");
		ss << line;
		ss >> week >> sow >> x >> y >> z >> temp >> temp >> temp
			>> temp >> temp >> temp >> temp
			>> temp >> temp >> temp
			>> temp >> temp >> temp
			>> temp >> temp >> temp
			>> temp >> temp >> temp
			>> yaw >> pitch >> roll;
		bool valid = true;
		Eigen::Vector3d xyz = Vector3d(x, y, z);
		Eigen::Vector3d vxyz = Vector3d(vx, vy, vz);
		Eigen::Matrix3d Rnb = a2mat(Vector3d(pitch, roll, -yaw) * M_PI / 180);

		if (!is_ref_set)
		{
			traj.ref = Vector3d(x, y, z);
			Rn0e = calcRne(traj.ref);
			is_ref_set = true;
		}
		Eigen::Vector3d twb = Rn0e * (xyz - traj.ref);
		Eigen::Vector3d vwb = Rn0e * vxyz;
		Eigen::Matrix3d Rn0b = Rn0e * calcRne(xyz).transpose() * Rnb;

		traj.poses[sow] = { Rn0b,twb,vwb, valid };
	}
	return traj;
}

inline Trajectory load_global_trajectory(const string& filename)
{
	Trajectory traj;
	string line;
	stringstream ss;
	ifstream ifs(filename);

	double sow, x, y, z, qw, qx, qy, qz;
	bool is_ref_set = false;
	Matrix3d Rn0e;
	while (ifs.good())
	{
		getline(ifs, line);
		if (line.size() < 1) continue;
		if (line[0] == '#') continue;
		ss.clear(); ss.str("");
		ss << line;
		ss >> sow >> x >> y >> z >> qw >> qx >> qy >> qz;
		bool valid = true;
		Eigen::Vector3d xyz = Vector3d(x, y, z);
		Eigen::Vector3d vxyz = Vector3d(0, 0, 0);
		Eigen::Matrix3d Rnb = Quaterniond(qw, qx, qy, qz).toRotationMatrix();

		if (!is_ref_set)
		{
			traj.ref = Vector3d(x, y, z);
			Rn0e = calcRne(traj.ref);
			is_ref_set = true;
		}
		Eigen::Vector3d twb = Rn0e * (xyz - traj.ref);
		Eigen::Vector3d vwb = Rn0e * vxyz;
		Eigen::Matrix3d Rn0b = Rn0e * calcRne(xyz).transpose() * Rnb;

		traj.poses[sow] = { Rn0b,twb,vwb, valid };
	}
	return traj;
}

inline Trajectory load_local_trajectory(const string& filename, const Vector3d& xyz_ref = Eigen::Vector3d::Zero())
{
	Trajectory traj;
	string line;
	stringstream ss;
	ifstream ifs(filename);

	traj.ref = xyz_ref;

	double sow, x, y, z, qw, qx, qy, qz;
	while (ifs.good())
	{
		getline(ifs, line);
		if (line.size() < 1) continue;
		if (line[0] == '#') continue;
		ss.clear(); ss.str("");
		ss << line;
		ss >> sow >> x >> y >> z >> qw >> qx >> qy >> qz;
		bool valid = true;
		traj.poses[sow] = { Quaterniond(qw,qx,qy,qz).toRotationMatrix(),Vector3d(x, y, z) ,Vector3d(0, 0, 0), valid };
	}
	return traj;
}

inline map<double, string> load_camstamp(const string & filename)
{
	map<double, string> camstamp;
	string line;
	stringstream ss;
	ifstream ifs(filename);

	double sow; string imagename;
	while (ifs.good())
	{
		getline(ifs, line);
		if (line.size() < 1) continue;
		if (line[0] == '#') continue;
		for (int ii = 0; ii < line.size(); ii++)
		{
			if (line[ii] == ',') line[ii] = ' ';
		}
		ss.clear(); ss.str("");
		ss << line;
		ss >> sow >> imagename;
		camstamp[sow] = imagename;
	}

	return camstamp;
}

