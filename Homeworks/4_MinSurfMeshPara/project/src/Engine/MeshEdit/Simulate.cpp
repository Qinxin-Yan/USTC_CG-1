#include <Engine/MeshEdit/Simulate.h>


#include <Eigen/Sparse>

using namespace Ubpa; 

using namespace std;
using namespace Eigen;


void Simulate::Clear() {
	this->positions.clear();
	this->velocity.clear();
}

bool Simulate::Init() {
	//Clear();

	
	this->velocity .resize(positions.size());
	for (int i = 0; i < positions.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			this->velocity[i][j] = 0;
		}
	}

	return true;
}

bool Simulate::Run() {
	SimulateOnce();

	// half-edge structure -> triangle mesh

	return true;
}

bool Simulate::IsFixed(unsigned id)
{
	for (int i = 0; i < fixed_id.size(); i++)
	{
		if (fixed_id[i] == id) return 1;
	}
	return 0;
}

void Simulate::SetSpringLenth()
{
	int s = edgelist.size() / 2;  //共s个弹簧
	spring_len.resize(s);
	for (int k = 0; k < s; k++)
	{
		auto i = edgelist[2 * k];
		auto j = edgelist[2 * k + 1];
		Vector3f d = { positions[j][0] - positions[i][0],positions[j][1] - positions[i][1],positions[j][2] - positions[i][2] };
		float len = d.norm();
		spring_len[k] = len;
		//cout << k << ":" << spring_len[k] << endl;
	}
}

void Simulate::SetLeftFix()
{
	//固定网格x坐标最小点
	fixed_id.clear();
	double x = 100000;
	for (int i = 0; i < positions.size(); i++)
	{
		if (positions[i][0] < x)
		{
			x = positions[i][0];
		}
	}

	for (int i = 0; i < positions.size(); i++)
	{
		if (abs(positions[i][0] - x) < 1e-5)
		{
			fixed_id.push_back(i);
		}
	}
}

void Simulate::SetTopFix()
{
	fixed_id.clear();
	double y = -100000;
	for (int i = 0; i < positions.size(); i++)
	{
		if (positions[i][1] > y)
		{
			y = positions[i][1];
		}
	}
	vector<int> tmp;
	for (int i = 0; i < positions.size(); i++)
	{
		if (abs(positions[i][1] - y) < 1e-5)
		{
			tmp.push_back(i);
		}
	}
	fixed_id.push_back(tmp[0]);
	fixed_id.push_back(tmp[tmp.size() - 1]);

	//Init();
}

void Simulate::SimulateOnce() {
	/*cout << times<<":" << endl;
	for (int i = 0; i < velocity.size(); i++)    //calculate gravity for each particle
	{
		cout << positions[i][0] << "  " << positions[i][1] << "  " << positions[i][2] << endl;
	}
	times++;

	vector<Vector3f> frc_list;                     //frc[i]为第i个质点的受力
	frc_list.resize(positions.size());

	//外力（重力）
	for (int i = 0; i < positions.size(); i++)    //calculate gravity for each particle
	{
		frc_list[i].setZero();
		if (!IsFixed(i))
		{
			frc_list[i].coeffRef(1, 0) -= mass_list[i] * gravity;
		}
		//frc_list[i].coeffRef(1,0) += mass_list[i] * gravity;
	}

	//内力（弹力和阻尼）
	int s = edgelist.size() / 2;  //共s个弹簧
	for (int k = 0; k < s; k++)
	{
		auto i = edgelist[2 * k];
		auto j = edgelist[2 * k + 1];
		Vector3f d = { positions[j][0] - positions[i][0],positions[j][1] - positions[i][1],positions[j][2] - positions[i][2] };
		Vector3f v = { velocity[j][0] - velocity[i][0],velocity[j][1] - velocity[i][1],velocity[j][2] - velocity[i][2] };
		float len = d.norm();
		//cout << len << endl;
		Vector3f f;
		//f = (stiff * (len / spring_len[k] - 1) + resistance / (len * spring_len[k]) * v.dot(d)) * d / len;
		f = (stiff * (len / spring_len[k] - 1.0)) * d / len;
		if (!IsFixed(i)) frc_list[i] += f;
		if (!IsFixed(j)) frc_list[j] -= f;
	}
	//欧拉半隐式求解
	for (int i = 0; i < positions.size(); i++)
	{
		//cout << positions[i][0] << "  " << positions[i][1] << "  " << positions[i][2] << endl;
		if (!IsFixed(i))
		{
			//cout << "i:" << i << endl;
			velocity[i] += { h * frc_list[i](0, 0), h * frc_list[i](1, 0), h * frc_list[i](2, 0) };
			positions[i] += {h* velocity[i][0], h* velocity[i][1], h* velocity[i][2]};
		}
	}*/
	FastSimulateOnce();
}

void Simulate::InitY()
{
	int m = positions.size();
	Y.resize(3 * m);
	for (int i = 0; i < positions.size(); i++)
	{
		Y.coeffRef(3 * i, 0) = positions[i][0];
		Y.coeffRef(3 * i + 1, 0) = positions[i][1];
		Y.coeffRef(3 * i + 2, 0) = positions[i][2];
	}
}

VectorXf Simulate::GetExtForce()  //gravity only
{
	//cout << "set external force!" << endl;
	VectorXf f;
	int m = positions.size();
	f.resize(3 * m);
	f.setZero();
	for (int i = 0; i < f.size(); i++)
	{
		if (i % 3 == 1) f.coeffRef(i, 0) = gravity;    
		//if (i % 3 == 0) f.coeffRef(i, 0) = gravity;
	}
	return f;
}

void Simulate::SetChol()
{
	cout << "setchol!" << endl;
	int m = positions.size();
	int s = edgelist.size() / 2;
	SparseMatrix<float> M;     //mass matrix
	M.resize(3 * m, 3 * m);
	M.setIdentity();
	//SparseMatrix<float> L;
	L.resize(3 * m, 3 * m);
	L.setZero();
	MatrixXf tmp;
	tmp.setZero(m,m);
	for (int i = 0; i < s; i++)
	{
		VectorXf A;
		A.resize(m);
		A.setZero();
		A.coeffRef(edgelist[2 * i],0) = 1;
		A.coeffRef(edgelist[2 * i + 1],0) = -1;
		tmp += stiff * A * A.transpose();
	}
	int row = 0;
	int col = 0;
	while (row < 3 * m)
	{
		col = 0;
		while (col < 3 * m)
		{
			L.coeffRef(row, col) = tmp(row / 3, col / 3);
			L.coeffRef(row + 1, col + 1) = tmp(row / 3, col / 3);
			L.coeffRef(row + 2, col + 2) = tmp(row / 3, col / 3);
			col = col + 3;
		}
		row = row + 3;
	}
	chol.compute(K*(M + h * h * L)*K.transpose());
}

void Simulate::SetK()
{
	int m = positions.size();
	int k = m - fixed_id.size();
	K.resize(3 * k, 3 * m);
	K.setZero();
	int count = 0;
	for (int i = 0; i < m; i++)
	{
		if (IsFixed(i)) count++;
		else
		{
			K.coeffRef(3 * (i - count), 3 * i) = 1;
			K.coeffRef(3 * (i - count) + 1, 3 * i + 1) = 1;
			K.coeffRef(3 * (i - count) + 2, 3 * i + 2) = 1;
		}
	}
}

void Simulate::SetJ()
{
	cout << "set J!" << endl;
	int m = positions.size();
	int s = edgelist.size() / 2;
	J.setZero(3 * m, 3 * s);
	SparseMatrix<float> M;     //mass matrix
	M.resize(3 * m, 3 * m);
	M.setIdentity();
	MatrixXf tmp;
	tmp.resize(m, s);
	tmp.setZero();
	for (int i = 0; i < s; i++)
	{
		VectorXf A;
		A.resize(m);
		A.setZero();
		A.coeffRef(edgelist[2 * i], 0) = 1;
		A.coeffRef(edgelist[2 * i + 1], 0) = -1;
		VectorXf S;
		S.resize(s);
		S.setZero();
		S.coeffRef(i, 0) = 1;
		tmp += stiff * A * S.transpose();
	}
	int row = 0;
	int col = 0;
	while (row < 3 * m)
	{
		col = 0;
		while (col < 3 * s)
		{
			J.coeffRef(row, col) = tmp(row / 3, col / 3);
			J.coeffRef(row + 1, col + 1) = tmp(row / 3, col / 3);
			J.coeffRef(row + 2, col + 2) = tmp(row / 3, col / 3);
			col = col + 3;
			//col = col + 3;
		}
		row = row + 3;
	}
}


Eigen::VectorXf Simulate::compute_d(Eigen::VectorXf X) 
{
	VectorXf d;
	int s = edgelist.size() / 2;
	d.resize(3 * s);
	for (int i = 0; i < edgelist.size()/2; i++)
	{
		int i1 = edgelist[2 * i];
		int i2 = edgelist[2 * i + 1];
		Vector3f p1 = { X(3 * i1,0),X(3 * i1 + 1,0),X(3 * i1 + 2,0) };
		Vector3f p2 = { X(3 * i2,0),X(3 * i2 + 1,0),X(3 * i2 + 2,0) };
		auto norm_ = (p1 - p2).norm();
		//cout << spring_len[i] << endl;
		d.segment(3 * i, 3) = spring_len[i] * (p1 - p2) / norm_;
	}
	return d;
}

void Simulate::FastSimulateOnce()
{
	times++;
	cout << times << endl;
	int m = positions.size();
	int s = edgelist.size() / 2;
	int k = m - fixed_id.size();
	VectorXf X=Y;   //positions
	VectorXf Xf;
	Xf.resize(3 * k);
	MatrixXf M;     //mass matrix
	M.setIdentity(3 * m, 3 * m);
	VectorXf d;     //spring 
	VectorXf ext_force = GetExtForce();   //external force (constant)
	VectorXf b;
	b.resize(3 * m);
	b = X - K.transpose() * K * X;

	//iteration
	cout << "iteration" << endl;
	for (int i = 1; i <= 3; i++)
	{
		d = compute_d(X);   //local, compute the optimal d
		Xf = chol.solve(K * (h * h * J * d - h * h * ext_force + M * Y - (M + h * h * L) * b)); // global step
		//Xf = chol.solve(K * (h * h * J * d + M * Y - (M + h * h * L) * b));
		X = K.transpose() * Xf + b;
	}

	//update positions & velocity
	cout << "update velocity!" << endl;
	for (int i = 0; i < positions.size(); i++)  //update positions
	{
		 positions[i] = { X(3 * i,0),X(3 * i + 1,0),X(3 * i + 2,0) };
	}
	for (int i = 0; i < velocity.size(); i++)   //update velocity
	{
		 velocity[i] = { (X(3 * i,0) - X_backup(3 * i,0)) / h,(X(3 * i,1) - X_backup(3 * i,1)) / h ,(X(3 * i,2) - X_backup(3 * i,2)) / h };
	}
	Y = 2 * X - X_backup;
	X_backup = X;  //update X_backup (X_n)
}