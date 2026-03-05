// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from olei_interfaces:msg/V2Header.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "olei_interfaces/msg/detail/v2_header__struct.h"
#include "olei_interfaces/msg/detail/v2_header__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool olei_interfaces__msg__v2_header__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("olei_interfaces.msg._v2_header.V2Header", full_classname_dest, 39) == 0);
  }
  olei_interfaces__msg__V2Header * ros_message = _ros_message;
  {  // magic
    PyObject * field = PyObject_GetAttrString(_pymsg, "magic");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->magic = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // version
    PyObject * field = PyObject_GetAttrString(_pymsg, "version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // distance_ratio
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_ratio");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->distance_ratio = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // brand
    PyObject * field = PyObject_GetAttrString(_pymsg, "brand");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_UINT8);
      Py_ssize_t size = 3;
      uint8_t * dest = ros_message->brand;
      for (Py_ssize_t i = 0; i < size; ++i) {
        uint8_t tmp = *(npy_uint8 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(uint8_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // vommercial
    PyObject * field = PyObject_GetAttrString(_pymsg, "vommercial");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_UINT8);
      Py_ssize_t size = 12;
      uint8_t * dest = ros_message->vommercial;
      for (Py_ssize_t i = 0; i < size; ++i) {
        uint8_t tmp = *(npy_uint8 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(uint8_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // internal
    PyObject * field = PyObject_GetAttrString(_pymsg, "internal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->internal = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // hardware
    PyObject * field = PyObject_GetAttrString(_pymsg, "hardware");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->hardware = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // software
    PyObject * field = PyObject_GetAttrString(_pymsg, "software");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->software = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // timestamp
    PyObject * field = PyObject_GetAttrString(_pymsg, "timestamp");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->timestamp = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // scan_frequency
    PyObject * field = PyObject_GetAttrString(_pymsg, "scan_frequency");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->scan_frequency = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // safe_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "safe_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->safe_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // error_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "error_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->error_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // status_flags
    PyObject * field = PyObject_GetAttrString(_pymsg, "status_flags");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->status_flags = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * olei_interfaces__msg__v2_header__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of V2Header */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("olei_interfaces.msg._v2_header");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "V2Header");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  olei_interfaces__msg__V2Header * ros_message = (olei_interfaces__msg__V2Header *)raw_ros_message;
  {  // magic
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->magic);
    {
      int rc = PyObject_SetAttrString(_pymessage, "magic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_ratio
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->distance_ratio);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_ratio", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // brand
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "brand");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_UINT8);
    assert(sizeof(npy_uint8) == sizeof(uint8_t));
    npy_uint8 * dst = (npy_uint8 *)PyArray_GETPTR1(seq_field, 0);
    uint8_t * src = &(ros_message->brand[0]);
    memcpy(dst, src, 3 * sizeof(uint8_t));
    Py_DECREF(field);
  }
  {  // vommercial
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "vommercial");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_UINT8);
    assert(sizeof(npy_uint8) == sizeof(uint8_t));
    npy_uint8 * dst = (npy_uint8 *)PyArray_GETPTR1(seq_field, 0);
    uint8_t * src = &(ros_message->vommercial[0]);
    memcpy(dst, src, 12 * sizeof(uint8_t));
    Py_DECREF(field);
  }
  {  // internal
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->internal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "internal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hardware
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->hardware);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hardware", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // software
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->software);
    {
      int rc = PyObject_SetAttrString(_pymessage, "software", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timestamp
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->timestamp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timestamp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // scan_frequency
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->scan_frequency);
    {
      int rc = PyObject_SetAttrString(_pymessage, "scan_frequency", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // safe_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->safe_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "safe_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // error_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->error_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "error_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // status_flags
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->status_flags);
    {
      int rc = PyObject_SetAttrString(_pymessage, "status_flags", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
