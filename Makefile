## hacky Makefile for robotpkg 
all:
	python setup.py build

install:
ifndef ROBOTPKG_BASE
	echo "environement varaible ROBOTPKG_BASE not defined"
else
	python dep.py
	python setup.py install --prefix ${ROBOTPKG_BASE}
endif