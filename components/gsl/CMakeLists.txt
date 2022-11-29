# Compiling and Linking the Library
# We have to make the component know where its .h files are
# and where to find its .a file.
# the GSL documentation says that for some functions both lgslcblas and lgsl must be linked in order to work properly
# (https://www.gnu.org/software/gsl/doc/html/usage.html#linking-programs-with-the-library)
 
idf_component_register( INCLUDE_DIRS ${COMPONENT_DIR}/include )
target_link_libraries( ${COMPONENT_LIB} INTERFACE ${COMPONENT_DIR}/lib/libgsl.a ${COMPONENT_DIR}/lib/libgslcblas.a)
 
add_prebuilt_library(gsl "libgsl.a libgslcblas.a")
#target_link_libraries( ${COMPONENT_LIB} INTERFACE ${COMPONENT_DIR}/lib/libgsl.a ${COMPONENT_DIR}/lib/libgslcblas.a)

# `main` calls a function from the library, so link it to `main`
#target_link_libraries(${COMPONENT_LIB} PRIVATE gls)