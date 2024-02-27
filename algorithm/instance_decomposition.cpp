#include "instance_decomposition.h"
//#include <Python.h>

namespace freeNav::LayeredMAPF {

    std::ostream &operator<<(std::ostream &os, std::set<int> agent_ids) {
        for (const auto &id : agent_ids) {
            os << id << " ";
        }
        return os;
    }

//    int runPython(const std::vector<std::set<int> >& agent_dependency_graph) {
//        Py_Initialize();
//        PyRun_SimpleString("import sys");
//        PyRun_SimpleString("sys.path.append('/home/yaozhuo/code/free-nav/scripts/')");
//        PyObject * pModule = PyImport_ImportModule("graph_visualize");
//        if (!pModule) {
//            printf("import python failed!!\n");
//            return -1;
//        }
//        PyObject* pFunc = PyObject_GetAttrString(pModule, "graph_visualize");
//
//        PyObject* pReturn = PyEval_CallObject(pFunc, NULL); //调用函数
//
//        //释放
//        if (pModule) {
//            Py_DECREF(pModule);
//        }
//        if (pFunc) {
//            Py_DECREF(pFunc);
//        }
//        if (pReturn) {
//            Py_DECREF(pReturn);
//        }
//        Py_Finalize();
//        return 0;
//    }


}
