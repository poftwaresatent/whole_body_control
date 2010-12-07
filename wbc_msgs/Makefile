include $(shell rospack find mk)/cmake.mk

mrp: clean
	svn st | awk '/^\?/{print $$2}' | xargs rm -rf
