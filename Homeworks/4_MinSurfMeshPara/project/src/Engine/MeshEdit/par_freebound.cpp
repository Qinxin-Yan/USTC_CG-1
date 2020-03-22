#include <Engine/MeshEdit/Paramaterize.h>
#include <Engine/MeshEdit/par_freebound.h>
#include <Engine/Primitive/TriMesh.h>

#include <math.h>
#include <Eigen/Sparse>
#include <Eigen/SVD>

using namespace Ubpa;
using namespace std;
using namespace Eigen;

Par_freebound::Par_freebound(Ptr<TriMesh> triMesh)
	: heMesh(make_shared<HEMesh<V>>())
{
	Init(triMesh);
}

void Par_freebound::Clear() {
	heMesh->Clear();
	triMesh = nullptr;
}

bool Par_freebound::Init(Ptr<TriMesh> triMesh) {
	Clear();

	if (triMesh == nullptr)
		return true;

	if (triMesh->GetType() == TriMesh::INVALID) {
		printf("ERROR::MinSurf::Init:\n"
			"\t""trimesh is invalid\n");
		return false;
	}

	// init half-edge structure
	size_t nV = triMesh->GetPositions().size();
	vector<vector<size_t>> triangles;
	triangles.reserve(triMesh->GetTriangles().size());
	for (auto triangle : triMesh->GetTriangles())
		triangles.push_back({ triangle->idx[0], triangle->idx[1], triangle->idx[2] });
	heMesh->Reserve(nV);
	heMesh->Init(triangles);

	if (!heMesh->IsTriMesh() || !heMesh->HaveBoundary()) {
		printf("ERROR::MinSurf::Init:\n"
			"\t""trimesh is not a triangle mesh or hasn't a boundaries\n");
		heMesh->Clear();
		return false;
	}

	// triangle mesh's positions ->  half-edge structure's positions
	for (int i = 0; i < nV; i++) {
		auto v = heMesh->Vertices().at(i);
		v->pos = triMesh->GetPositions()[i].cast_to<vecf3>();
	}

	this->triMesh = triMesh;
	return true;
}

bool Par_freebound::Run() {
	if (heMesh->IsEmpty() || !triMesh) {
		printf("ERROR::MinSurf::Run\n"
			"\t""heMesh->IsEmpty() || !triMesh\n");
		return false;
	}

	Para();

	// half-edge structure -> triangle mesh
	size_t nV = heMesh->NumVertices();
	size_t nF = heMesh->NumPolygons();
	vector<pointf3> positions;
	vector<unsigned> indice;
	positions.reserve(nV);
	indice.reserve(3 * nF);
	for (auto v : heMesh->Vertices())
		positions.push_back(v->pos.cast_to<pointf3>());
	for (auto f : heMesh->Polygons()) { // f is triangle
		for (auto v : f->BoundaryVertice()) // vertices of the triangle
			indice.push_back(static_cast<unsigned>(heMesh->Index(v)));
	}

	//triMesh->Init(indice, positions);

	return true;
}

void Par_freebound::SetxList()
{
	for (auto t : heMesh->Polygons())
	{
		vector<Vector2f> point_2D(3);
		vector<float> t_angle;

		vector<V*> point_3D = t->BoundaryVertice();
		if (point_3D.size() < 3) return;
		pointf3 p0 = { point_3D[0]->pos[0],point_3D[0]->pos[1],point_3D[0]->pos[2] };
		pointf3 p1 = { point_3D[1]->pos[0],point_3D[1]->pos[1],point_3D[1]->pos[2] };
		pointf3 p2 = { point_3D[2]->pos[0],point_3D[2]->pos[1],point_3D[2]->pos[2] };

		t_angle.push_back(get_angle(p0, p1, p2));
		t_angle.push_back(get_angle(p1, p0, p2));
		t_angle.push_back(get_angle(p2, p0, p1));
		x_angle.push_back(t_angle);

		point_2D[0] = { 0,0 };
		point_2D[1] = { distance(p0,p1),0 };
		point_2D[2] = { distance(p0,p1) * t_angle[0],distance(p0,p1) * sin_(t_angle[0]) };
		x_list.push_back(point_2D);
	}
}

/*void Par_freebound::SetMatList()
{
	//Q:返回的是不是所有的简单的面？
	//A:yes
	for (auto t : heMesh->Polygons())
	{
		vector<V*> point_3D = t->BoundaryVertice();
		Matrix2d mat;
		Vector2d u0(texcoords[Index(point_3D[0])][0], texcoords[Index(point_3D[0])][1]);
		Vector2d u1(texcoords[Index(point_3D[1])][0], texcoords[Index(point_3D[1])][1]);
		Vector2d u2(texcoords[Index(point_3D[2])][0], texcoords[Index(point_3D[2])][1]);
		vector<Vector2d> u_point = {u0,u1,u2};  //correct?
		size_t idx = Index(t);   //or set i=0...

		//set mat for face/polygon t
		for (int i = 0; i < 3; i++)
		{
			int j = (i + 1) % 3;
			mat += (x_angle[idx][i]) * ((u_point[i] - u_point[j]) * ((x_list[idx][i] - x_list[idx][j]).transpose()));
		}

		//find the best approximate, using SVD
		JacobiSVD<Matrix2d> svd(mat, ComputeFullU | ComputeFullV);
		Eigen::Matrix2d l, u, v;
		u= svd.matrixU();
		v = svd.matrixV();
		l = u * (v.transpose());

		mat_list.push_back(l);
	}
}*/

void Par_freebound::SetCoeffMatrix()
{
	SparseMatrix<float> co_mat(heMesh->NumVertices(), heMesh->NumVertices());
	//co_mat.resize(heMesh->NumVertices(), heMesh->NumVertices());
	//co_mat.setZero(heMesh->NumVertices(), heMesh->NumVertices());

	for (auto v : heMesh->Vertices())
	{
		auto idx_v = Index(v);
		auto adj_list = v->AdjVertices(); //get the adjvertexs
		double co_v = 0;

		for (auto adj_v : adj_list)
		{
			double co_adj = 0;
			auto idx_adjv = Index(adj_v);
			auto HE = v->HalfEdgeTo(adj_v);
			auto Poly = HE->Polygon();
			if (Poly != NULL)
			{
				auto idx_poly = Index(Poly);
				auto t = Poly->BoundaryVertice();
				int i;
				for (i = 0; i < 3; i++) //find the index of vertex in the triangle
				{
					if (t[i] != v && t[i] != adj_v) break;
				}
				//co_mat(idx_v, idx_v) += cot_(x_angle[idx_poly][i]);
				//co_v += cot_(x_angle[idx_poly][i]);
				co_adj += cot_(x_angle[idx_poly][i]);
			}

			HE = adj_v->HalfEdgeTo(v);
			Poly = HE->Polygon();
			if (Poly != NULL)
			{
				auto idx_poly = Index(Poly);
				auto t = Poly->BoundaryVertice();
				int i;
				for (i = 0; i < 3; i++)
				{
					if (t[i] != v && t[i] != adj_v) break;
				}
				//co_mat(idx_v, idx_v) += cot_(x_angle[idx_poly][i]);
				//co_v += cot_(x_angle[idx_poly][i]);
				co_adj += cot_(x_angle[idx_poly][i]);
				co_v += co_adj;
			}

			co_mat.coeffRef(idx_v, idx_adjv) = -1 * co_adj;
		}
		co_mat.coeffRef(idx_v, idx_v) = co_v;
	}

	chol.compute(co_mat);
}

void Par_freebound::InitTexcoords()
{
	auto bound = heMesh->Boundaries();
	vector<int> bound_idx;

	//get the index of the boundaries
	for (auto& group : bound)
	{
		for (auto ver : group)
		{
			bound_idx.push_back(heMesh->Index(ver->Pair()->End()));
		}
	}

	bound_idx.erase(std::unique(bound_idx.begin(), bound_idx.end()), bound_idx.end());
	std::sort(bound_idx.begin(), bound_idx.end());
	const auto& v = heMesh->Vertices();

	auto bound_size = bound_idx.size();
	auto len = ceil(bound_size / 4.0);

	//set parameter for the boundaries
	cout << "bound_size" << bound_size << endl;

	//auto bound_backup(bound);
	//bound_backup.assign(bound.begin(), bound.end());
	int i = 0;
	for (auto& group : bound)   //map the boundary of the curve to the unit square
	{
		//auto& group = bound[3];
		for (auto& p_v : group)
		{
			p_v->Pair()->End()->pos[2] = 0;
			if (i < len)
			{
				p_v->Pair()->End()->pos[0] = 0.0 + i * (1.0 / len);
				p_v->Pair()->End()->pos[1] = 0.0;
			}
			else if (i >= len && i < 2 * len)
			{
				p_v->Pair()->End()->pos[0] = 1.0;
				p_v->Pair()->End()->pos[1] = 0.0 + (i - len) * (1.0 / len);
			}
			else if (i >= 2 * len && i < 3 * len)
			{
				p_v->Pair()->End()->pos[0] = 1.0 - (i - 2 * len) * (1.0 / len);
				p_v->Pair()->End()->pos[1] = 1.0;
			}
			else if (i >= 3 * len)
			{
				p_v->Pair()->End()->pos[0] = 0.0;
				p_v->Pair()->End()->pos[1] = 1.0 - (i - 3 * len) * (1.0 / len);
			}

			i++;
		}
	}
	//cout << "i:" << i << endl;

	//set matrix 
	const auto mat_size = v.size() - bound_idx.size();
	SparseMatrix<double> mat(mat_size, mat_size);
	MatrixX2d right = Eigen::MatrixX2d::Zero(mat_size, 2);

	using std::cout;

	//auto v_backup=v;
	//v_backup.assign(v.begin(), v.end());
	for (auto vert : v)
	{
		int index = find_idx(bound_idx, heMesh->Index(vert));
		if (index != -1) //if vertex is not inthe boundaries
		{
			auto adj_v = vert->AdjVertices();  //get adjacent
			double degree = adj_v.size();
			if (index >= mat_size)
			{
				cout << "index=" << index << "mat_size=" << mat_size << endl;
				system("pause");
			}
			else
			{
				mat.coeffRef(index, index) += 1;
			}
			for (auto v : adj_v)
			{
				int row_idx = find_idx(bound_idx, heMesh->Index(v));
				if (row_idx == -1)
				{
					for (int i = 0; i < 2; i++)
					{
						right(index, i) += v->pos[i] / degree;
					}
				}
				else
				{
					if (index >= mat_size)
					{
						cout << "index:" << index << endl;
						system("pause");
					}
					else	if (row_idx >= mat_size)
					{
						cout << "row_index:" << row_idx << endl;
						system("pause");
					}
					else
					{
						mat.coeffRef(index, row_idx) -= 1 / degree;
					}
				}
			}
		}
	}

	SparseLU<SparseMatrix<double>> solver;
	solver.compute(mat);

	if (solver.info() != Success)
	{
		cerr << "Unable to decompose the matrix" << endl;
	}

	auto result = solver.solve(right);

	for (auto& vert : v)
	{
		int idx = find_idx(bound_idx, heMesh->Index(vert));
		if (idx != -1)
		{
			vert->pos[2] = 0;
			for (int i = 0; i < 2; i++)
			{
				vert->pos[i] = result(idx, i);
			}
		}
	}

	//update texcoords
	for (auto vert : v)
	{
		texcoords.push_back({ vert->pos[0],vert->pos[1] });
	}
}

MatrixXf Par_freebound::GetRightMatrix()
{
	auto num = heMesh->NumVertices();
	MatrixXf right(num, 2);
	right.setZero();
	auto v_list = heMesh->Vertices();

	for (auto v : heMesh->Vertices())
	{
		Vector2f temp_vct;
		temp_vct.setZero();

		auto idx_v = Index(v);
		auto adj_list = v->AdjVertices(); //get the adjvertexs
		float co_v = 0;

		for (auto adj_v : adj_list)
		{
			float co_adj = 0;
			auto idx_adjv = Index(adj_v);
			auto HE = v->HalfEdgeTo(adj_v);
			auto Poly = HE->Polygon();
			if (Poly != NULL)
			{
				auto idx_poly = Index(Poly);
				auto t = Poly->BoundaryVertice();
				int idx_i;
				int idx_j;
				int idx_theta;
				for (int i = 0; i < 3; i++) //find the index of vertex in the triangle
				{
					if (t[i] != v && t[i] != adj_v) idx_theta = i;
					if (t[i] == v) idx_i = i;
					if (t[i] == adj_v) idx_j = i;
				}

				temp_vct += cot_(x_angle[idx_poly][idx_theta]) * mat_list[idx_poly] * (x_list[idx_poly][idx_i] - x_list[idx_poly][idx_j]);
			}

			HE = adj_v->HalfEdgeTo(v);
			Poly = HE->Polygon();
			if (Poly != NULL)
			{
				auto idx_poly = Index(Poly);
				auto t = Poly->BoundaryVertice();
				int idx_i;
				int idx_j;
				int idx_theta;
				for (int i = 0; i < 3; i++) //find the index of vertex in the triangle
				{
					if (t[i] != v && t[i] != adj_v) idx_theta = i;
					if (t[i] == v) idx_i = i;
					if (t[i] == adj_v) idx_j = i;
				}

				temp_vct += cot_(x_angle[idx_poly][idx_theta]) * mat_list[idx_poly] * (x_list[idx_poly][idx_i] - x_list[idx_poly][idx_j]);
			}
		}
		right.coeffRef(idx_v, 0) = temp_vct(0);
		right.coeffRef(idx_v, 1) = temp_vct(1);
	}
	return right;
}


const int Par_freebound::find_idx(std::vector<int>& vec, int idx)
{
	auto N = vec.size();
	if (idx < vec[0]) return idx;
	if (idx > vec[N - 1]) return idx - N;
	for (auto i = 0; i < N; i++)
	{
		if (idx == vec[i]) return -1;
		if (idx < vec[i])
		{
			return idx - i;
		}
	}
}

float Par_freebound::distance(pointf3 p1, pointf3 p2)
{
	float temp = 0;
	for (int i = 0; i < 3; i++)
	{
		temp += pow(p1[i] - p2[i], 2);
	}
	return sqrt(temp);
}

float Par_freebound::get_angle(pointf3 p1, pointf3 p2, pointf3 p3)
{
	float d1 = distance(p1, p2);
	float d2 = distance(p2, p3);
	float d3 = distance(p3, p1);
	if (d1 == 0 || d3 == 0)
	{
		std::cout << "zero distance!" << endl;
		return -2;
	}
	return (pow(d1, 2) + pow(d3, 2) - pow(d2, 2)) / 2 * d1 * d3;
}

float Par_freebound::cot_(float cos_)
{
	if (sin_(cos_) == 0)
	{
		cout << "cos==0!" << endl;
		return -2;
	}
	return cos_ / sin_(cos_);
}

float Par_freebound::sin_(float cos_)
{
	return sqrt(1 - pow(cos_, 2));
}

void Par_freebound::SetMatList()
{
	for (auto t : heMesh->Polygons())
	{
		vector<V*> point_3D = t->BoundaryVertice();
		Matrix2f mat;
		Vector2f u0(texcoords[Index(point_3D[0])][0], texcoords[Index(point_3D[0])][1]);
		Vector2f u1(texcoords[Index(point_3D[1])][0], texcoords[Index(point_3D[1])][1]);
		Vector2f u2(texcoords[Index(point_3D[2])][0], texcoords[Index(point_3D[2])][1]);
		vector<Vector2f> u_point = { u0,u1,u2 };  //correct?
		size_t idx = Index(t);   //or set i=0... 

		//set mat for face/polygon t
		for (int i = 0; i < 3; i++)
		{
			int j = (i + 1) % 3;
			mat += (x_angle[idx][i]) * ((u_point[i] - u_point[j]) * ((x_list[idx][i] - x_list[idx][j]).transpose()));
		}

		//find the best approximate, using SVD
		JacobiSVD<Matrix2f> svd(mat, ComputeFullU | ComputeFullV);
		Eigen::Matrix2f l, u, v;
		u = svd.matrixU();
		v = svd.matrixV();
		l = u * (v.transpose());

		mat_list.push_back(l);
	}
}

void Par_freebound::Para()
{
	SetxList();
	SetCoeffMatrix();
	InitTexcoords();
	SetMatList();
	MatrixXd right = GetRightMatrix();

	auto tex = chol.solve(right);
	for (auto v : heMesh->Vertices())
	{
		int idx = Index(v);
		texcoords[idx][0] = tex(idx, 0);
		texcoords[idx][1] = tex(idx, 1);
	}

	triMesh->Update(texcoords);
}