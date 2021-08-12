#include <pybind11/pybind11.h>

namespace py = pybind11;

void add_dbtypes(py::module &m);

void add_db(py::module &m);

PYBIND11_MODULE(pylsfm, m) {
    add_dbtypes(m);
    add_db(m);
}
