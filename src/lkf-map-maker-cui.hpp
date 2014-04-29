/*
 * lkf-map-maker-cui.hpp
 *
 *  Created on: 2013/09/27
 *      Author: tyamada
 */

#ifndef LKF_MAP_MAKER_CUI_HPP_
#define LKF_MAP_MAKER_CUI_HPP_

#include "gnd-cui.hpp"

namespace lkf {
	namespace map_maker {

		static const gnd::cui_command cui_cmd[] = {
				{"Quit",					'Q',	"localizer shut-off"},
				{"help",					'h',	"show help"},
				{"show",					's',	"state show mode"},
				{"freq",					'f',	"change cycle (frequency)"},
				{"cycle",					'c',	"change cycle (cycle)"},
				{"start",					't',	"start (end stand-by mode)"},
				{"stand-by",				'B',	"stand-by mode"},
				{"", '\0'}
		};
	}
}

#endif /* LKF_MAP_MAKER_CUI_HPP_ */
