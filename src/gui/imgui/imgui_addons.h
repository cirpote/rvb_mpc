#pragma once

#ifndef IMGUI_API
#include "imgui.h"
#endif


namespace ImGui {

  // These 2 have a completely different implementation:
  // Posted by @JaapSuter and @maxint (please see: https://github.com/ocornut/imgui/issues/632)
  // Mod by @bartville removing the unreadable lambda function

  /*Usage Example

  const char* first = "a";
  const char* second = "b";
  const char* names[2];
  names[0] = first;
  names[1] = second;

  const float* couple_data[2];
  couple_data[0] = _x_values;
  couple_data[1] = _y_values;

  ImColor colorA = ImColor(255,0,0);
  ImColor colorB = ImColor(0,255,0);
  ImColor colori[2];
  colori[0] = colorA; 
  colori[1] = colorB;

  ImGui::PlotMultiLines("",
  			2,
  			names,
  			colori,
  			couple_data,
  			IM_ARRAYSIZE(_x_values),
  			_x_min,
  			_x_max,
  			ImVec2(0,40));

  */

  IMGUI_API void PlotMultiLines(
				const char* label,
				int num_datas,
				const char** names,
				const ImColor* colors,
				const float** values,
				int values_count,
				float scale_min,
				float scale_max,
				ImVec2 graph_size);
  
}
