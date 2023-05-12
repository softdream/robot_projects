#ifndef __GRID_CELL_H
#define __GRID_CELL_H

#include <cmath>
#include <type_traits>

namespace grid
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
struct GridCell
{
	GridCell() {  }
	~GridCell() {  } 
	
	explicit GridCell( const T log_odds_value ) : log_odds_value_( log_odds_value )
	{

	}

	bool isOccupied() const
	{
		return ( log_odds_value_ > 0.0 );
	}
	
	bool isFree() const
	{
		return ( log_odds_value_ < 0.0 );
	}

	bool isUnknow() const 
	{
		return ( log_odds_value_ == 0.0 );
	}
	
	void resetGridCell()
	{
		log_odds_value_ = 0.0;
	}

	T log_odds_value_ = 0.0;
	
	int update_index_ = 0;
};

template<typename T, typename = typename std::enable_if<is_double_or_float<T>::value>::type>
class GridCellOperations
{
public:
	using DataType = T;
	using GridCellType = GridCell<T>;	

	GridCellOperations()
	{
		log_odds_p_occ_ = probability2LogOdds( 0.6 );
        	log_odds_p_free_ = probability2LogOdds( 0.4 );
	}

	GridCellOperations( const DataType p_occ, const DataType p_free )
        {
                log_odds_p_occ_ = probability2LogOdds( p_occ );
                log_odds_p_free_ = probability2LogOdds( p_free );
        }
	

        ~GridCellOperations()
	{

	}

        const DataType getCellProbability( const GridCellType &cell ) const
	{
		DataType odds = ::exp( cell.log_odds_value_ );
		
		return ( odds / ( odds + 1.0 ) );
	}

        void setLogOddsPocc( const DataType p_occ )
	{
		log_odds_p_occ_ = probability2LogOdds( p_occ );
	}

        void setLogOddsPfree( const DataType p_free )
	{
		log_odds_p_free_ = probability2LogOdds( p_free );
	}

        const DataType getLogOddsPocc() const
	{
		return log_odds_p_occ_;
	}

        const DataType getLogOddsPfree() const
	{
		return log_odds_p_free_;
	}

        void setCellFree( GridCellType &cell ) const
	{
		cell.log_odds_value_ += log_odds_p_free_;
	}
	
        void setCellOccupied( GridCellType &cell ) const
	{
		cell.log_odds_value_ += log_odds_p_occ_;
	}

        void setCellUnFree( GridCellType &cell ) const
	{
		cell.log_odds_value_ -= log_odds_p_free_;
	}

private:
        const DataType probability2LogOdds( const DataType prob )
	{
		return ::log( prob / ( 1.0 - prob ) );		
	}

private:
        DataType log_odds_p_occ_;
        DataType log_odds_p_free_;
};

}

#endif
