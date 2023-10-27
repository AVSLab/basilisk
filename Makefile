.DEFAULT_GOAL = help

export SHELL := $(shell type --path bash)
export PYTHONPATH := $(shell pwd)/dist3:$(shell pwd)/src:${PYTHONPATH}

init: ## pip install conan
	pip install conan==1.59.0

build: ## build basilisk
	python3 conanfile.py

test: test-dist3 test-src ## test all

test-dist3: ## test dist3
	cd dist3 && ctest --build-config=Release

# pytest worker segfault
test-src: ## test src
	cd src && pytest --verbosity=1 --numprocesses=0

help: ## help
	-@grep --extended-regexp '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	| sed 's/^Makefile://1' \
	| awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-18s\033[0m %s\n", $$1, $$2}'

