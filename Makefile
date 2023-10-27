.DEFAULT_GOAL = help

export SHELL := $(shell type --path bash)

init: ## init venv for basilisk
	pip instal conan==1.59.0
	virtualenv basilisk
	source basilisk/bin/activate

build: ## build basilisk
	python3 conanfile.py

help: ## help
	-@grep --extended-regexp '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	| sed 's/^Makefile://1' \
	| awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-18s\033[0m %s\n", $$1, $$2}'

