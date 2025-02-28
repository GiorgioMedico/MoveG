all: prepare

install:
	sudo apt-get install gcc g++ cmake make doxygen git llvm pkg-config curl zip unzip tar python3-dev clang-format clang-tidy ccache ninja-build
	pip install cmake-format

prepare:
	rm -rf build
	mkdir build

coverage:
	sudo apt-get install lcov
	sudo pip3 install gcovr

dependency:
	cd build && cmake .. --graphviz=graph.dot && dot -Tpng graph.dot -o graphImage.png

pre-commit:
	pip3 install pre-commit
	pre-commit install
	pre-commit install-hooks
