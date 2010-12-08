EXTRA_CMAKE_FLAGS= -DWBC_ROOT=$(shell rospack find stanford_wbc)/local
include $(shell rospack find mk)/cmake.mk

wipe: clean
	rm -rf lib bin src/pr2_stanford_wbc

mrp: wipe
	svn st | awk '/^\?/{print $$2}' | xargs rm -rf
