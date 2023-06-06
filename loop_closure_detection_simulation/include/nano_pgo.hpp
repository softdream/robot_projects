#ifndef __NANO_PGO_HPP
#define __NANO_PGO_HPP

#include <type_traits>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

#include <Eigen/Sparse>
#include <Eigen/Eigen>

#include <Eigen/SparseCholesky>

#include <vector>

#include <memory>

namespace pgo
{

template<typename T>
struct is_double_or_float
{
        static const bool value = false;
};

template<>
struct is_double_or_float<float>
{
        static const bool value = true;
};

template<>
struct is_double_or_float<double>
{
        static const bool value = true;
};


template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
class GraphOptimizer
{
public:
	using DataType = T;
        using Vector3 = typename Eigen::Matrix<DataType, 3, 1>;
        using Matrix3x3 = typename Eigen::Matrix<DataType, 3, 3>;
        using Vector2 = typename Eigen::Matrix<DataType, 2, 1>;

	using ConstPtr = const std::unique_ptr<GraphOptimizer>;
	using Ptr = std::unique_ptr<GraphOptimizer>;

	GraphOptimizer()
	{ 

	}

	~GraphOptimizer()
	{

	}

	void addVertex( const Vector3 &pose, const int id )
	{
		vertex_ids.push_back( id );
		vertex_poses.push_back( pose );
	}

	void addEdge( const Vector3 &delta,
		      const int from,
		      const int to,
		      Matrix3x3 &info_matrix )
	{
		edge_from_ids.push_back( from );
		edge_to_ids.push_back( to );
		edge_means.push_back( delta );
		info_matrixes.push_back( info_matrix );
	}

	void execuGraphOptimization( const int max_iterations = 5 )
	{
		allocateMemory( vertex_poses.size() );

		for( size_t i = 0; i < vertex_poses.size(); i ++ ){
                        x( i * 3, 0 ) = vertex_poses[i](0);
                        x( i * 3 + 1, 0 ) = vertex_poses[i](1);
                        x( i * 3 + 2, 0 ) = vertex_poses[i](2);
                }

		for( int iteration = 0; iteration < max_iterations; iteration ++ ){
                        estimateOnce( );
                        std::cout<<"estimate iteration : "<<iteration<<std::endl;
                }

		for( size_t i = 0; i < vertex_poses.size(); i ++ ){
                        angleNormalize( vertex_poses[i](2) );
                       // DataType theta = x( i * 3 + 2 );
                       // DataType s = ::sin( theta );
                       // DataType c = ::cos( theta );
                       // x( i * 3 + 2 ) = ::atan2( s, c );
                }
	}
	
	const std::vector<Vector3>& getReultVertexPosesVector() const
	{
		return vertex_poses;
	}

private:
	void allocateMemory( const int vertex_num )
	{
		H.resize( vertex_num * 3, vertex_num * 3 );
                b.resize( vertex_num * 3, 1 );
                delta_x.resize( vertex_num * 3, 1 );
                x.resize( vertex_num * 3, 1 );
		
		std::cout<<"allocate the momory !"<<std::endl;
	}

	void estimateOnce()
	{
		getHessianDerived();
		
		// solve the linear system using sparse Cholesky factorization
                Eigen::SimplicialLDLT<Eigen::SparseMatrix<DataType>> solver;
                //Eigen::SparseLLT<Eigen::SparseMatrix<DataType>> solver;
                solver.compute( H.sparseView() );

                if (solver.info() != Eigen::Success) {
                        std::cerr << "Decomposition Failed !" << std::endl;
                        return;
                }

                delta_x.setZero();
                delta_x = solver.solve( b );
                if (solver.info() != Eigen::Success) {
                        std::cerr << "Solving Failed !" << std::endl;
                        return;
                }
		
		// update
                x += delta_x;

		//vertex_poses.clear();
                //for( size_t i = 0; i < x.size(); i += 3 ){
                //        Vector3 pose( x( i ), x( i + 1 ), x( i + 2 ) );
                //        vertex_poses.push_back( pose );
                //}

		for( size_t i = 0; i < vertex_poses.size(); i ++ ){
			vertex_poses[i](0) = x( 3 * i );
			vertex_poses[i](1) = x( 3 * i + 1 );
			vertex_poses[i](2) = x( 3 * i + 2 );
		}			
	}
	
	/*
	* caculate the Hessian Matrix
	*/
	void getHessianDerived()
	{
		H.setZero(); // 1. H <- 0
                b.setZero(); //    b <- 0
	
		// for all <e_ij, Omega_ij> do:
                for( size_t i = 0; i < edge_means.size(); i ++ ){
			int id_i = edge_from_ids[i];
                        int id_j = edge_to_ids[i];
			
			 // compute the Jacobians A_ij and B_ij of the error function
                        linearFactors( i );
		
			// compute the coefficient vector
                        b_i.setZero();
                        b_j.setZero();
                        H_ii.setZero();
                        H_ij.setZero();
                        H_ji.setZero();
                        H_jj.setZero();

                        Matrix3x3 omega = info_matrixes[i];

                        b_i = -A.transpose() * omega * e;
                        b_j = -B.transpose() * omega * e;
	
			// compute the contribution of this constraint to the linear system
                        H_ii = A.transpose() * omega * A;
                        H_ij = A.transpose() * omega * B;
                        //H_ji = B.transpose() * omega * A;
                        H_jj = B.transpose() * omega * B;

			H.block( id_i * 3, id_i * 3, 3, 3 ) += H_ii;
                        H.block( id_i * 3, id_j * 3, 3, 3 ) += H_ij;
                        H.block( id_j * 3, id_i * 3, 3, 3 ) += H_ij.transpose();
                        H.block( id_j * 3, id_j * 3, 3, 3 ) += H_jj;

                        b( id_i * 3, 0 ) += b_i( 0 );
                        b( id_i * 3 + 1, 0 ) += b_i( 1 );
                        b( id_i * 3 + 2, 0) += b_i( 2 );

                        b( id_j * 3, 0 ) += b_j( 0 );
                        b( id_j * 3 + 1, 0 ) += b_j( 1 );
                        b( id_j * 3 + 2, 0 ) += b_j( 2 );
		}
		
		H( 0, 0 ) = 1;
                H( 1, 1 ) = 1;
                H( 2, 2 ) = 1;	
	}

	/**
	* compute the taylor expansion of the error function of the index_th edge
	*/
	void linearFactors( const int index )
	{
		A.setZero();
		B.setZero();
		e.setZero();
		
		int id_i = edge_from_ids[index];
                int id_j = edge_to_ids[index];

		Vector3 v_i = vertex_poses[id_i];
                Vector3 v_j = vertex_poses[id_j];
                Vector3 z_ij = edge_means[index];

		Matrix3x3 zt_ij = v2t( z_ij );
                Matrix3x3 vt_i = v2t( v_i );
                Matrix3x3 vt_j = v2t( v_j );

		Matrix3x3 f_ij = vt_i.inverse() * vt_j;
	
		DataType theta_i = v_i[2];
                Vector2 t_i( v_i[0], v_i[1] );
                Vector2 t_j( v_j[0], v_j[1] );

                Vector2 dt_ij = t_j - t_i;

		DataType si = ::sin( theta_i );
                DataType ci = ::cos( theta_i );

		// caculate the Jaccobian Matrix
                A << -ci, -si, -si * dt_ij[0] + ci * dt_ij[1],
                      si, -ci, -ci * dt_ij[0] - si * dt_ij[1],
                       0,   0,  -1;

		B << ci, si, 0,
                    -si, ci, 0,
                      0,  0, 1;

		Matrix3x3 zt_ij_inv = zt_ij.inverse();

		e = t2v( zt_ij_inv * f_ij );
	
		zt_ij_inv( 0, 2 ) = 0;
                zt_ij_inv( 1, 2 ) = 0;

                A = zt_ij_inv * A;
                B = zt_ij_inv * B;
	}

public:	
	/**
	* vector to SE(2)
	*/
        const Matrix3x3 v2t( const Vector3 &v )
        {
                Matrix3x3 A;

                A << ::cos( v[2] ), -::sin( v[2] ), v[0],
                     ::sin( v[2] ),  ::cos( v[2] ), v[1],
                        0,                 0,        1;

                return A;
        }

	/*
	* SE(2) to vector
	*/
        const Vector3 t2v( const Matrix3x3 &A )
        {
                Vector3 v;

                v[0] = A( 0, 2 );
                v[1] = A( 1, 2 );
                v[2] = ::atan2( A( 1, 0 ), A( 0, 0 ) );

                return v;
        }

	void angleNormalize( DataType &angle )
        {
                if( angle >= M_PI ) {
                        angle -= 2 * M_PI;
                }

                if( angle <= -M_PI ){
                        angle += 2 * M_PI;
                }
        }

	const Vector3 homogeneousCoordinateTransformation( const Vector3& pose1, const Vector3& pose2 ) 	{
		Matrix3x3 T1 = v2t( pose1 );
		Matrix3x3 T2 = v2t( pose2 );

		Matrix3x3 Trans = T1.inverse() * T2;

		Vector3 V = t2v( Trans );
		return V;
	}

private:
	std::vector<int> vertex_ids;
	std::vector<Vector3> vertex_poses;
	std::vector<int> edge_from_ids;
	std::vector<int> edge_to_ids;
	std::vector<Vector3> edge_means;
	std::vector<Matrix3x3> info_matrixes;

	Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> H;

        Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> b;
        Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> delta_x;
        Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> x;

        Matrix3x3 A;
        Matrix3x3 B;
        Vector3 e;

        Vector3 b_i;
        Vector3 b_j;
        Matrix3x3 H_ii;
        Matrix3x3 H_ij;
        Matrix3x3 H_ji;
        Matrix3x3 H_jj;
};

}


#endif

