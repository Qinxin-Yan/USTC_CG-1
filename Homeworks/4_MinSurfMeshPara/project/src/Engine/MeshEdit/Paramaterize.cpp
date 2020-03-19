#include <Engine/MeshEdit/Paramaterize.h>

#include <Engine/MeshEdit/MinSurf.h>

#include <Engine/Primitive/TriMesh.h>

#include <math.h>

#include <Eigen/Sparse>
#include <MeshEdit\MinSurf.cpp>

using namespace Ubpa;

using namespace std;

using namespace Eigen;

Paramaterize::Paramaterize(Ptr<TriMesh> triMesh)
	: heMesh(make_shared<HEMesh<V>>())
{
	Init(triMesh);
}

void Paramaterize::Paramize() {
	//Ptr<TriMesh> triMesh_backup;
	//(*triMesh_backup) = (*triMesh);
	//const Ptr<HEMesh<V>> heMesh_backup;
	//(*heMesh_backup) = (*heMesh);
	cout << "paramize!" << endl;
	auto bound = heMesh->Boundaries();
	cout << "Number of boundaries:" << bound.size() << endl;
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

	auto bound_size=bound_idx.size();
	auto len = ceil(bound_size / 4.0);

	//set parameter for the boundaries
	cout << "bound_size" << bound_size << endl;

	//auto bound_backup(bound);
	//bound_backup.assign(bound.begin(), bound.end());
	int i = 0;
	for (auto& group : bound)   //map the boundary of the curve to the unit square
	{
		//auto& group = bound[3];
		for (auto &p_v : group)
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
	//cout << mat << endl;

	//cout << right_ << endl;

	SparseLU<SparseMatrix<double>> solver;
	solver.compute(mat);

	if (solver.info() != Success)
	{
		cerr << "Unable to decompose the matrix" << endl;
	}

	auto result = solver.solve(right);


	for (auto &vert : v)
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
	vector<pointf2> texcoords;
	for (auto vert : v)
	{
		texcoords.push_back({ vert->pos[0],vert->pos[1] });
	}
	triMesh->Update(texcoords);
	

	for (auto& group : bound)
	{
		for (auto& p_v : group)
		{
			cout << p_v->Pair()->End()->pos[0] << " " << p_v->Pair()->End()->pos[1] << " " << p_v->Pair()->End()->pos[2] << endl;
		}
	}

}

void Paramaterize::Clear() {
	heMesh->Clear();
	triMesh = nullptr;
}

bool Paramaterize::Init(Ptr<TriMesh> triMesh) {
	// TODO
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

bool Paramaterize::Run() {
	if (heMesh->IsEmpty() || !triMesh) {
		printf("ERROR::MinSurf::Run\n"
			"\t""heMesh->IsEmpty() || !triMesh\n");
		return false;
	}

	//cout << "paramize!" << endl;
	Paramize();

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

//return the index of the vertex in the matrix above
const int Paramaterize::find_idx(std::vector<int>& vec, int idx)
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
