#define PY_SSIZE_T_CLEAN
#include <Python.h>

PyObject *list;

list = Py_BuildValue("[iis]", 1, 2, "three");