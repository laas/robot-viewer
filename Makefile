## hacky Makefile for robotpkg 
all: build

build:
	python setup.py build

install: 
ifndef ROBOTPKG_BASE
	echo "environement varaible ROBOTPKG_BASE not defined"
else
	python setup.py install --prefix ${ROBOTPKG_BASE}
	sh makeaux.sh
endif

doc:
	cd docs && make html