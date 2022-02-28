/*
 * Copyright (C) 2020 Yirami.
 * All rights reserved.
 * 
 * @Brief: Simplify interface for packing / unpacking can messages.
 * @Author: [yirami.tang](https://yirami.xyz)
 * @Email: i@yirami.xyz
 * @Since: 2020-11-14 13:09:55
 * @LastEditors: Yirami
 * @LastEditTime: 2020-11-19 16:34:56
 * @Version: 
 */

#ifndef MODULES_PARSER_INCLUDE_CAN_PACKER_H_
#define MODULES_PARSER_INCLUDE_CAN_PACKER_H_

#ifdef __cplusplus
#include <cstring>  // strncpy
#include <string>
#include <array>
#include <map>

namespace yirami {
namespace utils {
namespace can {
namespace pack {
using std::string;
using std::array;
using std::map;

enum class PackStatus {
  SUCCESS = 0,
  INCOMPLETE,  // 存在未能打包的信号「ignore_err为真时有效」
  DBC_ERROR,  // 检查DBC文件是否存在、格式是否正确等
  MESSAGE_NOT_EXIST,  // DBC中未查到该报文
  SIGNAL_NOT_EXIST,  // DBC报文中未查到该信号
};

using CANData = array<unsigned char, 8>;
using CANFDData = array<unsigned char, 64>;
using SignalsPackage = map<string, double>;

PackStatus Init(const string &dbc_path);
const string& DBCVersion();

PackStatus Pack(const int &id, const SignalsPackage &sigs, CANData *raw,
                bool ignore_err = true, int *err_cnt = nullptr);
PackStatus Pack(const int &id, const SignalsPackage &sigs, CANFDData *raw,
                bool ignore_err = true, int *err_cnt = nullptr);
PackStatus Pack(const int &id, const SignalsPackage &sigs, unsigned char *raw,
                bool ignore_err = true, int *err_cnt = nullptr);

PackStatus UnPack(const int &id, const CANData &raw,
                  SignalsPackage *sigs);
PackStatus UnPack(const int &id, const CANFDData &raw,
                  SignalsPackage *sigs);
PackStatus UnPack(const int &id, const unsigned char *raw,
                  SignalsPackage *sigs);

PackStatus UnPack(const int &id, const CANData &raw, const string &name,
                  double *sig);
PackStatus UnPack(const int &id, const CANFDData &raw, const string &name,
                  double *sig);
PackStatus UnPack(const int &id, const unsigned char *raw, const string &name,
                  double *sig);

}  // namespace pack
}  // namespace can
}  // namespace utils
}  // namespace yirami

extern "C" {
#endif

#define Y_NAME_STRING_MAX_LENGTH      200

#define Y_CAN_PACK_SUCCESS            0
#define Y_CAN_PACK_INCOMPLETE         1
#define Y_CAN_PACK_DBC_ERROR          2
#define Y_CAN_PACK_MESSAGE_NOT_EXIST  3
#define Y_CAN_PACK_SIGNAL_NOT_EXIST   4

struct CSignalsPackage {
  int num;  // count the signals in the specified message
  char *name;  // Y_NAME_STRING_MAX_LENGTH
  double value;
};

int Init(const char *dbc_path);
int SignalsNum(int id);
const char* DBCVersion();

int Pack(int id, const CSignalsPackage *sigs, unsigned char *raw,
         bool ignore_err = true, int *err_cnt = nullptr);

int UnPack(int id, const unsigned char *raw, CSignalsPackage *sigs);

int UnPackSignal(int id, const unsigned char *raw, const char *name,
                 double *sig);

#ifdef __cplusplus
}
#endif


#endif  // MODULES_PARSER_INCLUDE_CAN_PACKER_H_
