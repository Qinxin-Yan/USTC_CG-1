#pragma once
#include <Basic/HeapObj.h>
#include <Engine/MeshEdit/MinSurf.h>
#include <Eigen/Sparse>
#include <math.h>
#include <Eigen/SVD>

namespace Ubpa {
	class TriMesh; 
	class MinSurf;

	//   mesh boundary == 1
	class Par_freebound : public HeapObj {
	public:
		Par_freebound(Ptr<TriMesh> triMesh);
	public:
		static const Ptr<Par_freebound> New(Ptr<TriMesh> triMesh) {
			return Ubpa::New<Par_freebound>(triMesh);
		}
	public:
		void Clear();
		bool Init(Ptr<TriMesh> triMesh);

		bool Run();
		bool Return();

	private:
		// kernel part of the algorithm
		void Para();

	private:
		float distance(pointf3 p1, pointf3 p2);  //get the distance from two points
		const int find_idx(std::vector<int>& vec, int idx);
		float get_angle(pointf3 p1, pointf3 p2, pointf3 p3);  //return the cos of p1
		float cot_(float cos_);
		float sin_(float cos_);

		Eigen::MatrixXf GetRightMatrix();   //set the right_matrix
		void SetCoeffMatrix();   //set the coeefient matrix and do cholesy, get chol;
		void SetxList();         //use trimesh to set x_list and x_angle;
		void InitTexcoords();    //init the texcoords using method in "paramaterize.h"
		void SetMatList();       //for each turn, set mat list (rotation or similarity)


	private:
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>>  chol;
		std::vector<std::vector<Eigen::Vector2f>> x_list;
		std::vector<std::vector<float>> x_angle;
		std::vector<Eigen::Matrix2f>   mat_list;  //the list of matrix in each turn
		std::vector<pointf2> texcoords;   //save the texcoords in order
		std::vector<pointf3> positions_backup;

	private:
		class V;
		class E;
		class P;
		class V : public TVertex<V, E, P> {
		public:
			vecf3 pos;
		};
		class E : public TEdge<V, E, P> { };
		class P :public TPolygon<V, E, P> { };

	private:
		friend class Par_ARAP;
		Ptr<TriMesh> triMesh;
		const Ptr<HEMesh<V>> heMesh; // vertice order is same with triMesh
	};
}
