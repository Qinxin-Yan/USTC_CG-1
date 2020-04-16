#pragma once

#include <Basic/HeapObj.h>
//#include <Engine/Primitive/MassSpring.h>
#include <UGM/UGM>
#include <Eigen/Sparse>
 
namespace Ubpa {
	class Simulate : public HeapObj {
	public:
		Simulate(const std::vector<pointf3>& plist,
			const std::vector<unsigned>& elist) {
			edgelist = elist;
			for (int i = 0; i < edgelist.size(); i++)
			{
				std::cout << edgelist[i] << std::endl;
			}
			gravity = 9.8;
			stiff = 10000;
			//spring_len = 1.0;
			this->positions.resize(plist.size());
			this->mass_list.resize(plist.size());
			for (int i = 0; i < plist.size(); i++)
			{
				mass_list[i] = 1;
				for (int j = 0; j < 3; j++)
				{
					this->positions[i][j] = plist[i][j];
				}
			}
			//SetLeftFix();
			SetTopFix();
			/*for (int i = 0; i < fixed_id.size(); i++)
			{
				std::cout << fixed_id[i] << std::endl;
			}*/
			SetK();
			SetSpringLenth();
			/*for (int i = 0; i < edgelist.size()/2; i++)
			{
				std::cout << "(" << edgelist[2 * i] << "," << edgelist[2 * i + 1] << ")" << std::endl;
			}*/
			InitY();
			X_backup = Y;
			SetChol();
			SetJ();
		};
	public:
		static const Ptr<Simulate> New(const std::vector<pointf3>& plist,
			const std::vector<unsigned> &elist) {
			return Ubpa::New<Simulate>(plist, elist);
		}
	public:
		// clear cache data
		void Clear();

		// init cache data (eg. half-edge structure) for Run()
		bool Init();
		//bool Init();

		// call it after Init()
		bool Run();
		
		const std::vector<pointf3>& GetPositions() const { return positions; };

		const float GetStiff() { return stiff; };
		void SetStiff(float k) { stiff = k; Init();};
		const float GetTimeStep() { return h; };
		void SetTimeStep(float k) { h = k; Init();};
		std::vector<unsigned>& GetFix() { return this->fixed_id; };
		void SetFix(const std::vector<unsigned>& f) { this->fixed_id = f; Init();};
		const std::vector<pointf3>& GetVelocity() { return velocity; };
		//void SetVelocity(const std::vector<pointf3>& v) { velocity = v; };

		void SetLeftFix();
		void SetTopFix();

		//float norm_(pointf3 p);
		bool IsFixed(unsigned id);
		void SetSpringLenth();  //set spring lenth according to the mesh


	private:
		// kernel part of the algorithm
		void SimulateOnce();


	private:
		void FastSimulateOnce();   //fast simulate of mass spring system
		//Eigen::VectorXf GetX(Eigen::VectorXf X1,Eigen::VectorXf X2);   //return y
		void InitY();
		void SetChol();
		void SetJ();
		Eigen::VectorXf compute_d( Eigen::VectorXf X);  //local part,commpute d
		Eigen::VectorXf GetExtForce();  //return extenal forces
		void SetK();   //set matrix K according to the fixed_id


	private:
		float h = 0.03f;  //步长
		float stiff=100000;
		float resistance=0;
		std::vector<float> spring_len;
		int times = 0;
		std::vector<unsigned> fixed_id;  //fixed point id

		Eigen::VectorXf Y;  //Y.size()=3*m;
		Eigen::VectorXf X_backup;
		Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>>  chol;
		Eigen::MatrixXf J;   //3m*3s
		Eigen::SparseMatrix<float> K;   //3k*3m
		Eigen::SparseMatrix<float> L;

		//mesh data
		std::vector<unsigned> edgelist;


		//simulation data
		float gravity=9.8;
		std::vector<float> mass_list;     //the same order as positions
		std::vector<pointf3> positions;    //位置
		std::vector<pointf3> velocity;    //速度
		
	};
}
