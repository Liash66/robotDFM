cmake_minimum_required(VERSION 3.5) # Проверка версии CMake.

set(PROJECT_NAME robotDFM)
project("${PROJECT_NAME}")

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)			 # устанавливаем 17 стандарт языка
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

include_directories(./opencv/include)
link_directories(./opencv/lib) # указываем, где искать файлы для opencv

include_directories(./asio/include)
link_directories(./asio/lib)		 # указываем, где искать файлы для asio

add_executable("${PROJECT_NAME}" main.cpp serial_connector/serial_connector.cpp)	# добавить исполняемый файл

target_link_libraries("${PROJECT_NAME}" optimized opencv_world490.lib)
target_link_libraries("${PROJECT_NAME}" debug opencv_world490d.lib)

file(COPY "res" DESTINATION "/")
file(COPY "opencv" DESTINATION "/")
file(COPY "asio" DESTINATION "/") 
file(COPY "serial_conncetor" DESTINATION "/") 