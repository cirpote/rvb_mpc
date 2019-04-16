add_library(dear_imgui SHARED
  gl3w.h
  gl3w.c
  imconfig.h
  imgui_draw.cpp
  imgui_impl_glfw_gl3.h
  stb_textedit.h
  imgui.cpp
  imgui.h
  imgui_internal.h
  stb_truetype.h
  imgui_demo.cpp
  imgui_impl_glfw_gl3.cpp
  stb_rect_pack.h
  imgui_addons.cpp # bart mods
  )

target_link_libraries(dear_imgui
  ${OPENGL_LIBRARY}
  glfw
  )
