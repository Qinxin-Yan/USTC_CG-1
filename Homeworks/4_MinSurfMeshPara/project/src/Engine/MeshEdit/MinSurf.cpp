#include <Engine/MeshEdit/MinSurf.h>

#include <Engine/Primitive/TriMesh.h>

#include <Eigen/Sparse>

using namespace Ubpa;

using namespace std;
using namespace Eigen;
 
MinSurf::MinSurf(Ptr<TriMesh> triMesh)
	: heMesh(make_shared<HEMesh<V>>())
{
	Init(triMesh);
}

void MinSurf::Clear() {
	heMesh->Clear();
	triMesh = nullptr;
}

bool MinSurf::Init(Ptr<TriMesh> triMesh) {
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

bool MinSurf::Run() {
	if (heMesh->IsEmpty() || !triMesh) {
		printf("ERROR::MinSurf::Run\n"
			"\t""heMesh->IsEmpty() || !triMesh\n");
		return false;
	}

	Minimize();

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

	triMesh->Init(indice, positions);

	return true;
}

void MinSurf::Minimize() {
	// TODO
	auto bound = heMesh->Boundaries();
	vector<int> bound_idx;
	//get the index of the boundary
	for (auto& ver : bound)
	{
		for (auto p_v : ver)
		{
			bound_idx.push_back(heMesh->Index(p_v->Pair()->End()));
		}
	}
	//sort
	bound_idx.erase(std::unique(bound_idx.begin(), bound_idx.end()), bound_idx.end());
	std::sort(bound_idx.begin(), bound_idx.end());
	const auto& v = heMesh->Vertices();

	const auto mat_size = v.size() - bound_idx.size();
	SparseMatrix<double> mat(mat_size, mat_size);
	//set matrix
	MatrixX3d right = Eigen::MatrixX3d::Zero(mat_size, 3);

	using std::cout;
	//get the matrix
	for (auto vert : v)
	{
		int index = find_idx(bound_idx, heMesh->Index(vert));
		if (index != -1) //if the vertex is not in the boundary
		{
			auto adj_v = vert->AdjVertices(); //get the adjvertexs
			double degree = adj_v.size();
			if (index >= mat_size)
			{
				cout << "index=" << index << "mat_size=" << mat_size << endl;
				system("pause");
			}
			else
			{
				mat.coeffRef(index, index) ++;
			}
			for (auto v : adj_v)
			{
				int row_idx = find_idx(bound_idx, heMesh->Index(v));
				if (row_idx == -1)
				{
					for (int i = 0; i < 3; i++)
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
	//solve matrix
	SparseLU<SparseMatrix<double>> solver;
	solver.compute(mat);

	if (solver.info() != Success)
	{
		cerr << "Unable to decompose the matrix" << endl;
	}

	auto result = solver.solve(right);
	//set vertex
	for (auto vert : v)
	{
		int idx = find_idx(bound_idx, heMesh->Index(vert));
		if (idx != -1)
		{
			for (int i = 0; i < 3; i++)
			{
				vert->pos[i] = result(idx, i);
			}
		}
	}
}

//return the index of the vertex in the matrix above
const int MinSurf::find_idx(std::vector<int>& vec, int idx)
{
	int N = vec.size();
	if (idx < vec[0]) return idx;
	if (idx > vec[N - 1]) return idx - N ;
	for (int i = 0; i < N; i++)
	{
		if (idx == vec[i]) return -1;
		if (idx < vec[i])
		{
			return idx - i;
		}
	}
}

