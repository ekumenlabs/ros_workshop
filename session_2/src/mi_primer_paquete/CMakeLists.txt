cmake_minimum_required(VERSION 3.0.2)
project(mi_primer_paquete)

## Compilar con soporte para C++11
add_compile_options(-std=c++11)

## Buscar catkin y sus componentes requeridos.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

########################################
## Configuracion especifica de catkin ##
########################################

## La macro catkin_package genera archivos de configuración cmake para tu paquete.
## Su fin es declarar cosas que se pasarán a proyectos dependientes.
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Especificar ubicaciones de archivos de encabezado.
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declarar un ejecutable de C++ (nodo).
add_executable(talker src/talker.cpp)

## Especificar bibliotecas con las que se debe linkear al ejecutable.
target_link_libraries(talker ${catkin_LIBRARIES})

## Declarar un ejecutable de C++ (nodo).
add_executable(listener src/listener.cpp)

## Especificar bibliotecas con las que se debe linkear al ejecutable.
target_link_libraries(listener ${catkin_LIBRARIES})
