#pragma once

#include <Basic/HeapObj.h>
#include <Engine/MeshEdit/MinSurf.h>
#include "MinSurf.h"

namespace Ubpa {
	class TriMesh;
	class MinSurf;

	//   mesh boundary == 1
	class Paramaterize : public HeapObj {
	public:
		Paramaterize(Ptr<TriMesh> triMesh);
	public:
		static const Ptr<Paramaterize> New(Ptr<TriMesh> triMesh) {
			return Ubpa::New<Paramaterize>(triMesh);
		}
	public:
		void Clear();
		bool Init(Ptr<TriMesh> triMesh);

		bool Run();

	private:
		// kernel part of the algorithm
		void Paramize();
		const int find_idx(std::vector<int>& vec, int idx);


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

		Ptr<TriMesh> triMesh;
		const Ptr<HEMesh<V>> heMesh; // vertice order is same with triMesh
	};
}
