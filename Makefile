file_finder = find . -type f $(1) -not \( -path './venv/*' -o -path './carla_msgs/*' -o -path './packaging/build/*' -o -path './build/*' -o -path './log/*' -o -path './install/*' \)

CMAKE_FILES = $(call file_finder,\( -name "*.cmake" -o -name "CMakeLists.txt" \))
PY_FILES = $(call file_finder,-name "*.py")

check: check_format pylint flake8

format:
	$(PY_FILES) | xargs autopep8 --in-place --max-line-length=100
	$(CMAKE_FILES) | xargs cmake-format -i

check_format:
	$(PY_FILES) | xargs autopep8 --diff --max-line-length=100
	$(CMAKE_FILES) | xargs cmake-format --check

pylint:
	$(PY_FILES) | xargs pylint --rcfile=.pylintrc

flake8:
	$(PY_FILES) | xargs flake8

deb:
	./packaging/build-deb.sh
