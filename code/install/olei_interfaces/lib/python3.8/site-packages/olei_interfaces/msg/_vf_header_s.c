// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from olei_interfaces:msg/VFHeader.idl
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
#include "olei_interfaces/msg/detail/vf_header__struct.h"
#include "olei_interfaces/msg/detail/vf_header__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool olei_interfaces__msg__vf_header__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("olei_interfaces.msg._vf_header.VFHeader", full_classname_dest, 39) == 0);
  }
  olei_interfaces__msg__VFHeader * ros_message = _ros_message;
  {  // magic
    PyObject * field = PyObject_GetAttrString(_pymsg, "magic");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->magic = (uint16_t)PyLong_AsUnsignedLong(field);
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
  {  // packet_size
    PyObject * field = PyObject_GetAttrString(_pymsg, "packet_size");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->packet_size = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // header_size
    PyObject * field = PyObject_GetAttrString(_pymsg, "header_size");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->header_size = (uint16_t)PyLong_AsUnsignedLong(field);
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
  {  // types
    PyObject * field = PyObject_GetAttrString(_pymsg, "types");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->types = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // scan_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "scan_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->scan_number = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // packet_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "packet_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->packet_number = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // timestamp_decimal
    PyObject * field = PyObject_GetAttrString(_pymsg, "timestamp_decimal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->timestamp_decimal = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // timestamp_integer
    PyObject * field = PyObject_GetAttrString(_pymsg, "timestamp_integer");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->timestamp_integer = PyLong_AsUnsignedLong(field);
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
  {  // num_points_scan
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_points_scan");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_points_scan = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // iutput_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "iutput_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->iutput_status = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // output_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "output_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->output_status = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // field_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "field_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->field_status = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // start_index
    PyObject * field = PyObject_GetAttrString(_pymsg, "start_index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->start_index = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // end_index
    PyObject * field = PyObject_GetAttrString(_pymsg, "end_index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->end_index = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // first_index
    PyObject * field = PyObject_GetAttrString(_pymsg, "first_index");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->first_index = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_points_packet
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_points_packet");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_points_packet = (uint16_t)PyLong_AsUnsignedLong(field);
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
PyObject * olei_interfaces__msg__vf_header__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of VFHeader */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("olei_interfaces.msg._vf_header");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "VFHeader");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  olei_interfaces__msg__VFHeader * ros_message = (olei_interfaces__msg__VFHeader *)raw_ros_message;
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
  {  // packet_size
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->packet_size);
    {
      int rc = PyObject_SetAttrString(_pymessage, "packet_size", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // header_size
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->header_size);
    {
      int rc = PyObject_SetAttrString(_pymessage, "header_size", field);
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
  {  // types
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->types);
    {
      int rc = PyObject_SetAttrString(_pymessage, "types", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // scan_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->scan_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "scan_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // packet_number
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->packet_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "packet_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timestamp_decimal
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->timestamp_decimal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timestamp_decimal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timestamp_integer
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->timestamp_integer);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timestamp_integer", field);
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
  {  // num_points_scan
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_points_scan);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_points_scan", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // iutput_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->iutput_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "iutput_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // output_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->output_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "output_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // field_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->field_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "field_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // start_index
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->start_index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "start_index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // end_index
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->end_index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "end_index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // first_index
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->first_index);
    {
      int rc = PyObject_SetAttrString(_pymessage, "first_index", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_points_packet
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_points_packet);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_points_packet", field);
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
