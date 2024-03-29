/*
 * lkf-map-maker-conf.hpp
 *
 *  Created on: 2013/09/27
 *      Author: tyamada
 */

#ifndef LKF_MAP_MAKER_CONF_HPP_
#define LKF_MAP_MAKER_CONF_HPP_


#include <ssmtype/spur-odometry.h>
#include "ssm-laser.hpp"

#include "gnd-lkf.hpp"
#include "gnd-config-file.hpp"
#include "gnd-lib-error.h"


// ---> constant definition
namespace lkf {
	namespace map_maker {
		static const char proc_name[] = "lkf-map-maker";

		// map-file
		static const gnd::conf::parameter_array<char, 256> ConfIni_InitialMapDir = {
				"init-lkf-map",
				"",		// map file directory
				"input scan matching map directory path (optional argument)"
		};

		// ssm-name
		static const gnd::conf::parameter_array<char, 256> ConfIni_PositionSSMName = {
				"position-ssm-name",
				"spur_adjust",
				"position ssm name"
		};

		// ssm-id
		static const gnd::conf::parameter<int> ConfIni_PositionSSMID = {
				"position-ssm-id",
				0,
				"position ssm id"
		};

		// laser scanner ssm-name
		static const gnd::conf::parameter_array<char, 256> ConfIni_LaserScannerSSMName = {
				"laser-scanner-ssm-name",
				"scan_data2d",
				"laser scanner ssm name"
		};

		// laser scanner ssm-name
		static const gnd::conf::parameter<int> ConfIni_LaserScannerSSMID = {
				"laser-scanner-ssm-id",
				0,
				"laser scanner ssm id"
		};


		// ndt
		static const gnd::conf::parameter<bool> ConfIni_NDT = {
				"ndt",
				false,
				"scan matching evaluation function (ndt or lkf)"
		};

		// culling threshold
		static const gnd::conf::parameter<double> ConfIni_Culling = {
				"culling",
				gnd_m2dist( 0.05 ),	// [m]
				"distance threshold of scan data culling ([m])"
		};

		//
		static const gnd::conf::parameter<double> ConfIni_LaserUseDist = {
				"use-range-dist",
				gnd_m2dist( 20 ),	// [m]
				"distance threshold of use laser scanner data for scan matching ([m])"
		};

		//
		static const gnd::conf::parameter<double> ConfIni_LaserUseOrient = {
				"use-range-angle",
				gnd_deg2ang(-1),	// [deg]
				"orientation threshold of use laser scanner data for scan matching ([deg])"
		};



		// rest-cycle
		static const gnd::conf::parameter<double> ConfIni_PauseCycle = {
				"pause-time",
				gnd_sec2time(10.0),	// [s]
				"pause scan matching during this time, when robot is stopping ([sec])"
		};
		// rest-threshold-distance
		static const gnd::conf::parameter<double> ConfIni_PauseDist = {
				"pause-distance",
				gnd_m2dist(0.10),	// [m]
				"pause scan matching during robot move less than this parameter ([m])"
		};
		// rest-threshold-orientation
		static const gnd::conf::parameter<double> ConfIni_PauseOrient = {
				"pause-orientation",
				gnd_deg2ang(45),	// [rad]
				"pause scan matching during robot move less than this parameter ([deg])"
		};



		//
		static const gnd::conf::parameter_array<char, 256> ConfIni_LKFMap = {
				"lkf-map",
				"lkf-map",		// path name
				"lkf map output directory"
		};

		//
		static const gnd::conf::parameter<double> ConfIni_LKFCounterCellSize = {
				"counter-cell-size",
				1.0,
				"counter cell size"
		};


		// additional smoothing parameter
		static const gnd::conf::parameter<double> ConfIni_LKFAddSmoothParam = {
				"additional-smoothing",
				10,		// number of additional reflection point for each cells
				"parameter for additional smoothing (number of additional reflection point for each cells)"
		};

		// bmp
		static const gnd::conf::parameter<bool> ConfIni_BMP = {
				"bmp-map",
				false, 		// true of false
				"scan matching map file with BMP (for human)"
		};

		// bmp
		static const gnd::conf::parameter<double> ConfIni_BMPPixelSize = {
				"bmp-map-pixel-size",
				0.05,
				"BMP pixel size"
		};

		// laser point log
		static const gnd::conf::parameter_array<char, 256> ConfIni_LaserPointLog = {
				"laser-point-txtlog",
				"",		// file name
				"laser point log file name (text)"
		};


		// file output directory
		static const gnd::conf::parameter_array<char, 256> ConfIni_OutputDir = {
				"file-output-directory",
				"./",		// directory path
				"file output directory"
		};


		// occupancy-grid-map
		static const gnd::conf::parameter<bool> ConfIni_Occupancy = {
				"occupancy-grid-map",
				false,		// true of false
				"occupancy grid map with BMP"
		};

		// occupancy-grid-map-pixel-size
		static const gnd::conf::parameter<double> ConfIni_OccPixelSize = {
				"occupancy-grid-map-pixel-size",
				0.05,		// file name
				"pixel size of occupancy grid map ([m])"
		};

		// occupancy-grid-map-occupied-probability
		static const gnd::conf::parameter<double> ConfIni_OccOccupiedProb = {
				"occupied-probability",
				0.51,		// probability
				"probability that laser reflected space is occupied (for occupancy grid map)"
		};

		// occupancy-grid-map-free-probability
		static const gnd::conf::parameter<double> ConfIni_OccFreeProb = {
				"free-probability",
				0.6,		// probability
				"probability that laser passing space is free (for occupancy grid map)"
		};

		// occupancy-grid-map-free-probability
		static const gnd::conf::parameter<double> ConfIni_OccMeasurementError = {
				"sensor-measurement-error",
				0.15,		// [m]
				"measurement error ([m]) (for occupancy grid map)"
		};

	} // <--- namespace lkf
} // <--- namespace position_tracker
// <--- constant definition


// ---> type declaration
namespace lkf {
	namespace map_maker {
		struct proc_configuration;
		typedef struct proc_configuration proc_configuration;
	}
}
// ---> type declaration


// ---> function declaration
namespace lkf {
	namespace map_maker {
		/*
		 * @brief initialize configure to default parameter
		 */
		int proc_conf_initialize(proc_configuration *conf);

		/*
		 * @brief get configuration parameter
		 */
		int proc_conf_get( gnd::conf::configuration* src, proc_configuration* dest );
		/*
		 * @brief set configuration parameter
		 */
		int proc_conf_set( gnd::conf::configuration* dest, proc_configuration* src );

		/*
		 * @brief read configuration parameter
		 */
		int proc_conf_read( const char* f, proc_configuration* dest );
		/*
		 * @brief write configuration parameter
		 */
		int proc_conf_write( const char* f, proc_configuration* src );

	}
}
// ---> function declaration



// ---> type definition
namespace lkf {
	namespace map_maker {
		/**
		 * \brief particle localizer configure
		 */
		struct proc_configuration {
			proc_configuration();

			gnd::conf::parameter_array<char, 256>	init_map;				///< scan matching map directory path
			gnd::conf::parameter_array<char, 256>	pos_name;				///< odometry position estimation log file name
			gnd::conf::parameter<int>				odm_id;					///< corrected position log id
			gnd::conf::parameter_array<char, 256>	ls_name;				///< laser scanner log file name
			gnd::conf::parameter<int>				ls_id;					///< corrected position log id

			gnd::conf::parameter<double>			culling;				///< laser scanner data decimate parameter [m]
			gnd::conf::parameter<double>			range_dist;				///< matching data parameter (distance threshold)
			gnd::conf::parameter<double>			range_orient;			///< matching data parameter (orient threshold)

			gnd::conf::parameter<double>			pause_time;				///< resting mode cycle
			gnd::conf::parameter<double>			pause_dist;				///< resting threshold (position distance) [m]
			gnd::conf::parameter<double>			pause_orient;			///< resting threshold (position orientation) [deg]

			gnd::conf::parameter<bool>				ndt;					///< ndt mode
			gnd::conf::parameter_array<char, 256>	lkf_map;				///< lkf map directory
			gnd::conf::parameter<double>			lkf_countercellsize;	///< counter cell size
			gnd::conf::parameter<double>			lkf_addsmoothparam;		///< lkf map directory
			gnd::conf::parameter<bool>				bmp;					///< lkf map (bmp)
			gnd::conf::parameter<double>			bmp_pixelsize;			///< bmp map pixel size

			gnd::conf::parameter<bool>				occ_map;				///< occupancy grid map
			gnd::conf::parameter<double>			occ_pixelsize;			///< occupancy grid map pixel size
			gnd::conf::parameter<double>			occ_occupiedprob;		///< occupied probability
			gnd::conf::parameter<double>			occ_freeprob;			///< free probability
			gnd::conf::parameter<double>			occ_measurementerror;	///< measurement error

			gnd::conf::parameter_array<char, 256>	laserpoint_log;			///< laser point log

			gnd::conf::parameter_array<char, 256>	output_dir;				///< file output directory
		};

		/**
		 * @brief constructor
		 */
		inline
		proc_configuration::proc_configuration(){
			proc_conf_initialize(this);
		}

	}
}
// ---> type definition




// ---> function definition
namespace lkf {
	namespace map_maker {

		/**
		 * @brief initialize configure
		 */
		inline
		int proc_conf_initialize(proc_configuration *conf){
			gnd_assert(!conf, -1, "invalid null pointer");

			::memcpy(&conf->pos_name,				&ConfIni_PositionSSMName,		sizeof(ConfIni_PositionSSMName) );
			::memcpy(&conf->odm_id,					&ConfIni_PositionSSMID,			sizeof(ConfIni_PositionSSMID) );
			::memcpy(&conf->ls_name,				&ConfIni_LaserScannerSSMName,	sizeof(ConfIni_LaserScannerSSMName) );
			::memcpy(&conf->ls_id,					&ConfIni_LaserScannerSSMID,		sizeof(ConfIni_LaserScannerSSMID) );
			::memcpy(&conf->init_map,				&ConfIni_InitialMapDir,			sizeof(ConfIni_InitialMapDir) );
			::memcpy(&conf->culling,				&ConfIni_Culling,				sizeof(ConfIni_Culling) );
			::memcpy(&conf->range_dist,				&ConfIni_LaserUseDist,			sizeof(ConfIni_LaserUseDist) );
			::memcpy(&conf->range_orient,			&ConfIni_LaserUseOrient,		sizeof(ConfIni_LaserUseOrient) );
			::memcpy(&conf->pause_time,				&ConfIni_PauseCycle,			sizeof(ConfIni_PauseCycle) );
			::memcpy(&conf->pause_dist,				&ConfIni_PauseDist,				sizeof(ConfIni_PauseDist) );
			::memcpy(&conf->pause_orient,			&ConfIni_PauseOrient,			sizeof(ConfIni_PauseOrient) );

			::memcpy(&conf->lkf_map,				&ConfIni_LKFMap,				sizeof(ConfIni_LKFMap) );
			::memcpy(&conf->lkf_countercellsize,	&ConfIni_LKFCounterCellSize,	sizeof(ConfIni_LKFCounterCellSize) );
			::memcpy(&conf->lkf_addsmoothparam,		&ConfIni_LKFAddSmoothParam,		sizeof(ConfIni_LKFAddSmoothParam) );
			::memcpy(&conf->bmp,					&ConfIni_BMP,					sizeof(ConfIni_BMP) );
			::memcpy(&conf->bmp_pixelsize,			&ConfIni_BMPPixelSize,			sizeof(ConfIni_BMPPixelSize) );
			::memcpy(&conf->ndt,					&ConfIni_NDT,					sizeof(ConfIni_NDT) );

			::memcpy(&conf->occ_map,				&ConfIni_Occupancy,				sizeof(ConfIni_Occupancy) );
			::memcpy(&conf->occ_pixelsize,			&ConfIni_OccPixelSize,			sizeof(ConfIni_OccPixelSize) );
			::memcpy(&conf->occ_occupiedprob,		&ConfIni_OccOccupiedProb,		sizeof(ConfIni_OccOccupiedProb) );
			::memcpy(&conf->occ_freeprob,			&ConfIni_OccFreeProb,			sizeof(ConfIni_OccFreeProb) );
			::memcpy(&conf->occ_measurementerror,	&ConfIni_OccMeasurementError,	sizeof(ConfIni_OccMeasurementError) );

			::memcpy(&conf->laserpoint_log,			&ConfIni_LaserPointLog,			sizeof(ConfIni_LaserPointLog) );
			::memcpy(&conf->output_dir,				&ConfIni_OutputDir,				sizeof(ConfIni_OutputDir) );

			return 0;
		}

		/**
		 * @brief get configuration parameter
		 * @param  [in] src  : configuration
		 * @param [out] dest : configuration parameter
		 */
		inline
		int proc_conf_get( gnd::conf::configuration* src, proc_configuration* dest )
		{
			gnd_assert(!src, -1, "invalid null pointer");
			gnd_assert(!dest, -1, "invalid null pointer");

			gnd::conf::get_parameter( src, &dest->init_map );
			gnd::conf::get_parameter( src, &dest->culling );
			gnd::conf::get_parameter( src, &dest->range_dist );
			if( !gnd::conf::get_parameter( src, &dest->range_orient) )
				dest->range_orient.value = gnd_deg2ang(dest->range_orient.value);
			gnd::conf::get_parameter( src, &dest->pause_time );
			gnd::conf::get_parameter( src, &dest->pause_dist );
			if( !gnd::conf::get_parameter( src, &dest->pause_orient ) )
				dest->pause_orient.value = gnd_deg2ang(dest->pause_orient.value);

			gnd::conf::get_parameter( src, &dest->lkf_map );
			gnd::conf::get_parameter( src, &dest->lkf_countercellsize );
			gnd::conf::get_parameter( src, &dest->lkf_addsmoothparam );
			gnd::conf::get_parameter( src, &dest->ndt );
			gnd::conf::get_parameter( src, &dest->bmp );
			gnd::conf::get_parameter( src, &dest->bmp_pixelsize );

			gnd::conf::get_parameter( src, &dest->occ_map );
			gnd::conf::get_parameter( src, &dest->occ_pixelsize );
			gnd::conf::get_parameter( src, &dest->occ_occupiedprob );
			gnd::conf::get_parameter( src, &dest->occ_freeprob );
			gnd::conf::get_parameter( src, &dest->occ_measurementerror );

			gnd::conf::get_parameter( src, &dest->pos_name );
			gnd::conf::get_parameter( src, &dest->odm_id );
			gnd::conf::get_parameter( src, &dest->ls_name );
			gnd::conf::get_parameter( src, &dest->ls_id );

			gnd::conf::get_parameter( src, &dest->laserpoint_log );
			gnd::conf::get_parameter( src, &dest->output_dir );

			return 0;
		}

		/**
		 * @brief file out  configure file
		 */
		inline
		int proc_conf_set( gnd::conf::configuration* dest, proc_configuration* src ) {

			gnd_assert(!dest, -1, "invalid null pointer");
			gnd_assert(!src, -1, "invalid null pointer");

			{ // ---> operation
				gnd::conf::set_parameter(dest, &src->pos_name);
				gnd::conf::set_parameter(dest, &src->odm_id);
				gnd::conf::set_parameter(dest, &src->ls_name);
				gnd::conf::set_parameter(dest, &src->ls_id);

				// scan matching parameter
				gnd::conf::set_parameter(dest, &src->culling);

				gnd::conf::set_parameter(dest, &src->pause_time);
				gnd::conf::set_parameter(dest, &src->pause_dist);
				src->pause_orient.value = gnd_ang2deg(src->pause_orient.value);
				gnd::conf::set_parameter(dest, &src->pause_orient );
				src->pause_orient.value = gnd_deg2ang(src->pause_orient.value);

				gnd::conf::set_parameter(dest, &src->range_dist);
				src->range_orient.value = gnd_ang2deg(src->range_orient.value);
				gnd::conf::set_parameter(dest, &src->range_orient);
				src->range_orient.value = gnd_deg2ang(src->range_orient.value);

				gnd::conf::set_parameter(dest, &src->laserpoint_log );
				gnd::conf::set_parameter(dest, &src->output_dir );

				gnd::conf::set_parameter(dest, &src->lkf_map );
				gnd::conf::set_parameter(dest, &src->lkf_countercellsize );
				gnd::conf::set_parameter(dest, &src->lkf_addsmoothparam );
				gnd::conf::set_parameter(dest, &src->ndt );
				gnd::conf::set_parameter(dest, &src->bmp );
				gnd::conf::set_parameter(dest, &src->bmp_pixelsize );

				gnd::conf::set_parameter(dest, &src->occ_map );
				gnd::conf::set_parameter(dest, &src->occ_pixelsize );
				gnd::conf::set_parameter(dest, &src->occ_occupiedprob );
				gnd::conf::set_parameter(dest, &src->occ_freeprob );
				gnd::conf::set_parameter(dest, &src->occ_measurementerror );

				gnd::conf::set_parameter(dest, &src->init_map);

				return 0;
			} // <--- operation
		}


        /**
         * @brief read configuration parameter file
         * @param [in]  f    : configuration file name
         * @param [out] dest : configuration parameter
         */
        inline
        int proc_conf_read(const char* f, proc_configuration* dest) {
                gnd_assert(!f, -1, "invalid null pointer");
                gnd_assert(!dest, -1, "invalid null pointer");

                { // ---> operation
                        int ret;
                        gnd::conf::file_stream fs;
                        // configuration file read
                        if( (ret = fs.read(f)) < 0 )    return ret;

                        return proc_conf_get(&fs, dest);
                } // <--- operation
        }

        /**
         * @brief write configuration parameter file
         * @param [in]  f  : configuration file name
         * @param [in] src : configuration parameter
         */
        inline
        int proc_conf_write(const char* f, proc_configuration* src){
                gnd_assert(!f, -1, "invalid null pointer");
                gnd_assert(!src, -1, "invalid null pointer");

                { // ---> operation
                        int ret;
                        gnd::conf::file_stream fs;
                        // convert configuration declaration
                        if( (ret = proc_conf_set(&fs, src)) < 0 ) return ret;

                        return fs.write(f);
                } // <--- operation
        }


	} // namespace position_tracker
}; // namespace lkf
// <--- function definition


#endif /* LKF_MAP_MAKER_CONF_HPP_ */
