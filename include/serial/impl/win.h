/********************************************//**
 *
 * This provides a windows implementation of the Serial class interface.
 *
 ***********************************************/

#ifdef _WIN32

#ifndef SERIAL_IMPL_WINDOWS_H
#define SERIAL_IMPL_WINDOWS_H

#include "serial.h"

#include "windows.h"

namespace serial {

using std::string;
using std::wstring;
using std::invalid_argument;

using serial::SerialException;
using serial::IOException;

class serial::Serial::SerialImpl {
public:
  SerialImpl (const string &port,
              unsigned long baudrate,
              bytesize_t bytesize,
              parity_t parity,
              stopbits_t stopbits,
              flowcontrol_t flowcontrol);

  virtual ~SerialImpl ();

  void
  open ();

  void
  close ();

  bool
  isOpen () const;

  size_t
  available ();

  size_t
  read (uint8_t *buf, size_t size = 1);

  size_t
  write (const uint8_t *data, size_t length);

  void
  flush ();

  void
  flushInput ();

  void
  flushOutput ();

  void
  sendBreak (int duration);

  void
  setBreak (bool level);

  void
  setRTS (bool level);

  void
  setDTR (bool level);

  bool
  waitForChange ();

  bool
  getCTS ();

  bool
  getDSR ();

  bool
  getRI ();

  bool
  getCD ();

  void
  setPort (const string &port);

  string
  getPort () const;

  void
  setTimeout (Timeout &timeout);

  Timeout
  getTimeout () const;

  void
  setBaudrate (unsigned long baudrate);

  unsigned long
  getBaudrate () const;

  void
  setBytesize (bytesize_t bytesize);

  bytesize_t
  getBytesize () const;

  void
  setParity (parity_t parity);

  parity_t
  getParity () const;

  void
  setStopbits (stopbits_t stopbits);

  stopbits_t
  getStopbits () const;

  void
  setFlowcontrol (flowcontrol_t flowcontrol);

  flowcontrol_t
  getFlowcontrol () const;

  void
  readLock ();

  void
  readUnlock ();

  void
  writeLock ();

  void
  writeUnlock ();

  //expose for use with WaitForMultiplePorts

  HANDLE fd_;

  //the handles to be able to read and write asyncronously
  OVERLAPPED read_asynchronous;
  OVERLAPPED write_asynchronous;
protected:
  void reconfigurePort ();

private:
  wstring port_;               // Path to the file descriptor

  bool is_open_;

  Timeout timeout_;           // Timeout for read operations
  unsigned long baudrate_;    // Baudrate

  parity_t parity_;           // Parity
  bytesize_t bytesize_;       // Size of the bytes
  stopbits_t stopbits_;       // Stop Bits
  flowcontrol_t flowcontrol_; // Flow Control

  // Mutex used to lock the read functions
  HANDLE read_mutex;
  // Mutex used to lock the write functions
  HANDLE write_mutex;

  
};

}

#endif // SERIAL_IMPL_WINDOWS_H

#endif // _WIN32
