project(stlplus)

FILE(GLOB LIBSLTPLUS_HPP *.hpp )
FILE(GLOB LIBSLTPLUS_CPP *.cpp )

ADD_LIBRARY(stlplus SHARED ${LIBSLTPLUS_HPP} ${LIBSLTPLUS_CPP})
