#ifndef TRACKER_H
#define TRACKER_H

#include "discriminator/KalmanFilter.h"
#include "discriminator/kalmanFilterVideo.h"
#include "discriminator/AuctionAlgorithm.h"

using namespace Eigen;

template<size_t StateDim, size_t MeasurementDim>
class Tracker
{
public:

    typedef kalmanFilterVideo Filter;
    typedef typename Matrix<double, MeasurementDim, 1> MeasurementSpaceVector;
    
//     typedef typename Filter::StateSpaceVector StateSpaceVector;
//     typedef typename Filter::MeasurementSpaceVector MeasurementSpaceVector;
//     typedef typename Filter::MeasurementMatrix MeasurementMatrix;
//     typedef typename Filter::MeasurementStateConversionMatrix MeasurementStateConversionMatrix;
//     typedef typename Filter::StateMatrix StateMatrix;
    
    typedef std::vector<MeasurementDim> Measurements;
    typedef std::vector<Filter> Filters;

    Tracker() 
    {
      // Inizializzazione dei parametri di Kalman
      _dt = 0.033;
      _stateCovariance << 50, 50;
      _measurePosCovariance << 0.6667, 0.6667;
      _measureVelCovariance << 6.6667, 6.6667;
      _initialState << w/2, 0, h/2, 0;
    }
    
    private:
        
        float _dt;
        
        Matrix<float, StateDim, StateDim> _stateCovariance;

        Matrix<float, MeasurementDim, MeasurementDim> _measurePosCovariance, _measureVelCovariance;

        MatrixXf<float StateDim, 1> _initialState;

        Filters _filters;

    const Filters & filters() const { return _filters; }

    void track(const Measurements &measurements)
    {
        const size_t m = measurements.size();
        const size_t f = _filters.size();

        // create matrix for calculating distances between measurements and predictions
        // additional rows for initializing filters (weightet by 1 / (640 * 480))
        MatrixXd w_ij(m, f + m);

        w_ij = MatrixXd::Zero(m, f + m);

        // get likelihoods of measurements within track pdfs
        for ( size_t i = 0; i < m; ++i )
        {
            for ( size_t j = 0; j < f; ++j )
                w_ij(i, j) = _filters[j].likelihood(measurements[i]);
        }

        // weights for initializing new filters
        for ( size_t j = f; j < m + f; ++j )
            w_ij(j - f, j) = 1. / (640. * 480. );

        // solve the maximum-sum-of-weights problem (i.e. assignment problem)
        // in this case it is global nearest neighbour by minimizing the distances
        // over all measurement-filter-associations
        // TODO apply AUCTION ALGORITHM
        //Auction<double>::Edges assignments = Auction<double>::solve(w_ij);
        Auction<double>::Edges assignments;
        
        Filters newFilters;

        // for all found assignments
        for ( const auto & e : assignments )
        {
            // is assignment an assignment from an already existing filter to a measurement?
            if ( e.y < f )
            {
                // update filter and keep it
                _filters[e.y].update(measurements[e.x]);
                newFilters.emplace_back(_filters[e.y]);
            }
            else // is this assignment a measurement that is considered new?
            {
                // create filter with measurment and keep it
                
                Filter newFilter(_dt, _stateCovariance, 
                                 _measurePosCovariance, _measureVelCovariance, _initialState);
                newFilter.initializeFilter();
                // Filter newFilter(measurements[e.x], _F, _Q, _R, _H);
                newFilters.emplace_back(newFilter);
            }
        }

        // current filters are now the kept filters
        _filters = newFilters;
    }
};

#endif // TRACKER_H
