## hacky Makefile for robotpkg 
all: build

build:
	python setup.py build

install: 
ifndef LOCALBASE
	echo "LOCALBASE not defined"
else
	python setup.py install --prefix ${LOCALBASE}
	make doc
	sh makeaux.sh
endif

doc:
	cd docs && make html