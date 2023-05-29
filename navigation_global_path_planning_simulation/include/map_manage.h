#ifndef __MAP_MANAGE_H
#define __MAP_MANAGE_H

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include "occupied_grid_map.h"

namespace map
{

template<typename T>
class MapManagement
{
public:
	static bool saveOccupiedGridMap( const std::string &file_name, const grid::OccupiedGridMap<T> &map );

	static const grid::OccupiedGridMap<T> loadOccupiedGridMap( const std::string &file_name );

	static void displayOccupiedGridMap( grid::OccupiedGridMap<T> &map );

private:
	static std::ofstream outfile;	
	static std::ifstream infile;
};

template<typename T>
std::ofstream MapManagement<T>::outfile;

template<typename T>
std::ifstream MapManagement<T>::infile;

template<typename T>
bool MapManagement<T>::saveOccupiedGridMap( const std::string &file_name, const grid::OccupiedGridMap<T> &map )
{
	outfile.open( file_name, std::ios::binary | std::ios::out );
	if( !outfile.is_open() ){
		std::cerr<<"Failed to open the map file !"<<std::endl;
		return false;
	}
	std::cerr<<"Open the Map File !"<<std::endl;

	// 1. write the map information
	int size_x = map.getSizeX();
	int size_y = map.getSizeY();
	Eigen::Vector2i center = map.getMapCenter();
	int center_x = center[0];
	int center_y = center[1];
	T cell_length = map.getCellLength();
	T map_scale = map.getScale();
	
	outfile.write( reinterpret_cast<char *>( &size_x ), sizeof( size_x ) );
	outfile.write( reinterpret_cast<char *>( &size_y ), sizeof( size_y ) );
	outfile.write( reinterpret_cast<char *>( &center_x ), sizeof( center_x ) );
	outfile.write( reinterpret_cast<char *>( &center_y ), sizeof( center_y ) );
	outfile.write( reinterpret_cast<char *>( &cell_length ), sizeof( cell_length ) );
	outfile.write( reinterpret_cast<char *>( &map_scale ), sizeof( map_scale ) );
	std::cout<<"write the map information !"<<std::endl;
	
	// 2. write the grid cell operations information
	T log_odds_p_occ = map.getLogOddsPoccValue();
	T log_odds_p_free = map.getLogOddsPfreeValue();

	outfile.write( reinterpret_cast<char *>( &log_odds_p_occ ), sizeof(log_odds_p_occ ) );
	outfile.write( reinterpret_cast<char *>( &log_odds_p_free ), sizeof(log_odds_p_free ) );
	std::cout<<"write the cell information !"<<std::endl;

	// 3. write the Occupied Grid Map information
	int curr_update_index = map.getCurrUpdateIndex();
	int curr_mark_occ_index = map.getCurrMarkOccIndex();
	int curr_mark_free_index = map.getCurrMarkFreeIndex();
	outfile.write( reinterpret_cast<char *>( &curr_update_index ), sizeof( curr_update_index ) );
	outfile.write( reinterpret_cast<char *>( &curr_mark_occ_index ), sizeof( curr_mark_occ_index ) );
	outfile.write( reinterpret_cast<char *>( &curr_mark_free_index ), sizeof( curr_mark_free_index ) );	
	std::cout<<"write the grid map information !"<<std::endl;

	// 3. write the GridCell information
	std::vector<grid::GridCell<T>> map_array = map.getMapArray();
	outfile.write( reinterpret_cast<char *>( map_array.data() ), map_array.size() * sizeof( grid::GridCell<T> ) );
	std::cout<<"write the grid cells !"<<std::endl;
	
	outfile.close();
	return true;
}

template<typename T>
const grid::OccupiedGridMap<T> MapManagement<T>::loadOccupiedGridMap( const std::string &file_name )
{
	infile.open( file_name, std::ios::binary );
	if( !infile.is_open() ){
		std::cerr<<"Failed to Open the Map File !"<<std::endl;
		exit(-1);
	}
	std::cerr<<"Open the Map File !"<<std::endl;

	grid::OccupiedGridMap<T> map;

	// 1. read the map information
	int size_x = -1;
	int size_y = -1;
	Eigen::Vector2i center( -1, -1 );
	int center_x = -1;
	int center_y = -1;
	T cell_length = 0.0;
	T map_scale = 0.0;
	
	infile.read( reinterpret_cast<char *>( &size_x ), sizeof( size_x ) );
	infile.read( reinterpret_cast<char *>( &size_y ), sizeof( size_y ) );
	infile.read( reinterpret_cast<char *>( &center_x ), sizeof( center_x ) );
	infile.read( reinterpret_cast<char *>( &center_y ), sizeof( center_y ) );
	infile.read( reinterpret_cast<char *>( &cell_length ), sizeof( cell_length ) );
	infile.read( reinterpret_cast<char *>( &map_scale ), sizeof( map_scale ) );
	std::cout<<"Read the map information !"<<std::endl;
	center = Eigen::Vector2i( center_x, center_y );
	
	map.setMapInfo( size_x, size_y, cell_length );

	// 2. read the GridCell Operations information
	T log_odds_p_occ = 0.0;
        T log_odds_p_free = 0.0;

	infile.read( reinterpret_cast<char *>( &log_odds_p_occ ), sizeof( log_odds_p_occ ) );
	infile.read( reinterpret_cast<char *>( &log_odds_p_free ), sizeof( log_odds_p_free ) );
	std::cout<<"read the cell information !"<<std::endl;

	map.setLogOddsPoccValue( log_odds_p_occ );
	map.setLogOddsPfreeValue( log_odds_p_free );

	// 3. read the Occupied Grid Map information
	int curr_update_index = -1;
        int curr_mark_occ_index = -1;
        int curr_mark_free_index = -1;

	infile.read( reinterpret_cast<char *>( &curr_update_index ), sizeof( curr_update_index ) );
        infile.read( reinterpret_cast<char *>( &curr_mark_occ_index ), sizeof( curr_mark_occ_index ) );
        infile.read( reinterpret_cast<char *>( &curr_mark_free_index ), sizeof( curr_mark_free_index ) );
        std::cout<<"read the grid map information !"<<std::endl;

	// 4. read the GridCell information
	std::vector<grid::GridCell<T>> map_array;
	map_array.resize( size_x * size_y );
	infile.read( reinterpret_cast<char *>( map_array.data() ), map_array.size() * sizeof( grid::GridCell<T> ) );
	std::cout<<"read the grid cells !"<<std::endl;

	map.setMapArray( map_array );

	infile.close();
	
	return map;
}

template<typename T>
void MapManagement<T>::displayOccupiedGridMap( grid::OccupiedGridMap<T> &map )
{
	cv::Mat image = cv::Mat::zeros( map.getSizeX(), map.getSizeY(), CV_8UC3);	
	int occupiedCount = 0;
        for( int i = 0; i < map.getSizeX(); i ++ ){
        	for( int j = 0; j < map.getSizeY(); j ++ ){
                	if( map.isCellFree( i, j ) ){
                        	cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(255, 255, 255), -1);
                        }
                        else if( map.isCellOccupied( i, j ) ){
                                occupiedCount ++;
                                cv::circle(image, cv::Point2d(i, j), 1, cv::Scalar(0, 0, 255), -1);
                        }
                }
	}

	std::cout<<"occupied grid number : "<<occupiedCount<<std::endl;
        cv::imshow( "map", image );
	cv::waitKey( 0 );
}

}

#endif
