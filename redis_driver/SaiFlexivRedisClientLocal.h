#ifndef SAI_FLEXIV_REDIS_CLIENT_LOCAL_H
#define SAI_FLEXIV_REDIS_CLIENT_LOCAL_H

#include <Eigen/Core>
#include <hiredis/hiredis.h>
#include <cmath>
#include <string>
#include <iostream>
#include <stdexcept>
#include <json/json.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <array>
#include <vector>

namespace Sai {
namespace Flexiv {

using std::cout;
using std::string;
using std::runtime_error;

inline const char *redisReplyTypeName(int type) {
	switch (type) {
	case REDIS_REPLY_STRING:
		return "string";
	case REDIS_REPLY_ARRAY:
		return "array";
	case REDIS_REPLY_INTEGER:
		return "integer";
	case REDIS_REPLY_NIL:
		return "nil";
	case REDIS_REPLY_STATUS:
		return "status";
	case REDIS_REPLY_ERROR:
		return "error";
	default:
		return "unknown";
	}
}

inline std::string redisContextError(const redisContext *context) {
	if (context == nullptr) {
		return "Redis context is null";
	}
	if (context->err != 0) {
		return std::string("hiredis error: ") + context->errstr;
	}
	return "hiredis returned REDIS_ERR without a context error";
}

inline std::string redisReplySummary(const redisReply *reply) {
	if (reply == nullptr) {
		return "Redis returned a null reply";
	}

	std::stringstream ss;
	ss << "Redis reply type " << redisReplyTypeName(reply->type)
	   << " (" << reply->type << ")";
	if (reply->str != nullptr && reply->len > 0) {
		const size_t max_payload_len = 256;
		const size_t payload_len = std::min(reply->len, max_payload_len);
		ss << ": " << std::string(reply->str, payload_len);
		if (reply->len > max_payload_len) {
			ss << "...";
		}
	}
	return ss.str();
}

inline void checkRedisReply(redisReply *reply, const redisContext *context,
							const std::string &operation) {
	if (reply == nullptr) {
		throw runtime_error(operation + " failed: " + redisContextError(context));
	}
	if (reply->type == REDIS_REPLY_ERROR) {
		throw runtime_error(operation + " failed: " + redisReplySummary(reply));
	}
}

inline void checkRedisGetReplyStatus(int status, const redisContext *context,
									 const std::string &operation) {
	if (status == REDIS_ERR) {
		throw runtime_error(operation + " failed: " + redisContextError(context));
	}
}

inline bool parseFiniteDoubleToken(const std::string &token, double &value) {
	try {
		size_t parsed_len = 0;
		value = std::stod(token, &parsed_len);
		return parsed_len == token.size() && std::isfinite(value);
	} catch (const std::exception &) {
		return false;
	}
}

struct HiredisServerInfo {
public:
    string hostname_;
    int port_;
    timeval timeout_;
};

class CDatabaseRedisClient {
public:
	CDatabaseRedisClient ()
		: context_(NULL),
		reply_(NULL)
	{// do nothing
	}

	~CDatabaseRedisClient() {
		freeCurrentReply();
		if (context_ != NULL) {
			redisFree(context_);
			context_ = NULL;
		}
	}

	CDatabaseRedisClient(const CDatabaseRedisClient &) = delete;
	CDatabaseRedisClient &operator=(const CDatabaseRedisClient &) = delete;

	// init with server info
	void serverIs(HiredisServerInfo server) {
		// delete existing connection
		if (NULL != context_) {
			redisFree(context_);
			context_ = NULL;
		}
		if (server.hostname_.empty()) {
			// nothing to do
			return;
		}
		// set new server info
		server_info_ = server;
		// connect to new server
		auto tmp_context = redisConnectWithTimeout(server.hostname_.c_str(), server.port_, server.timeout_);
        if (NULL == tmp_context) {
        	throw(runtime_error("Could not allocate redis context."));
        }
    	if (tmp_context->err) {
			std::string err = string("Could not connect to redis server : ") + string(tmp_context->errstr);
			redisFree(tmp_context);
			throw(runtime_error(err.c_str()));
        }
		if (redisSetTimeout(tmp_context, server.timeout_) != REDIS_OK) {
			std::string err = string("Could not set redis command timeout : ") +
							  string(tmp_context->errstr);
			redisFree(tmp_context);
			throw(runtime_error(err.c_str()));
		}
    	// set context, ping server
    	context_ = tmp_context;
	}

public:
	// set expiry (ms) on an existing db key
	void keyExpiryIs(const string& key, const uint expiry_ms) {
		const std::string operation = operationForKey("Redis PEXPIRE", key);
		ensureConnected(operation);
		reply_ = (redisReply *)redisCommand(context_, "PEXPIRE %s %s", key.c_str(), std::to_string(expiry_ms).c_str());
		if (NULL == reply_) {
			throw(runtime_error(operation + " failed: " + redisContextError(context_)));
		}
		if (REDIS_REPLY_ERROR == reply_->type) {
			const std::string error = redisReplySummary(reply_);
			freeCurrentReply();
			throw(runtime_error(operation + " failed: " + error));
		}
		freeCurrentReply();
	}

	bool getCommandIs(const string &cmd_mssg) {
		const std::string operation = operationForKey("Redis GET", cmd_mssg);
		ensureConnected(operation);
		reply_ = (redisReply *)redisCommand(context_, "GET %s", cmd_mssg.c_str());
		if (NULL == reply_ || REDIS_REPLY_ERROR == reply_->type) {
			std::string error = redisContextError(context_);
			if (reply_ != NULL) {
				error = redisReplySummary(reply_);
				freeCurrentReply();
			}
			throw(runtime_error(operation + " failed: " + error));
		}
		if (REDIS_REPLY_NIL == reply_->type) {
			// cout << "\nNo data on server.. Missing key?";
			return false;
		}
		if (REDIS_REPLY_STRING != reply_->type) {
			const std::string error = redisReplySummary(reply_);
			freeCurrentReply();
			throw(runtime_error(operation + " returned unexpected data: " +
								error));
		}
		return true;
	}

	bool getCommandIs(const string &cmd_mssg, string &ret_string) {
		const bool success = getCommandIs(cmd_mssg);
		if (!success) {
			freeCurrentReply();
			return false;
		}
		ret_string.assign(reply_->str, reply_->len);
		freeCurrentReply();
		return true;
	}

	bool getCommandIs(const string &cmd_mssg, double &ret_double) {
		const bool success = getCommandIs(cmd_mssg);
		if (!success) {
			freeCurrentReply();
			return false;
		}
		const std::string payload(reply_->str, reply_->len);
		freeCurrentReply();
		if (!parseFiniteDoubleToken(payload, ret_double)) {
			throw(runtime_error(operationForKey("Redis GET", cmd_mssg) +
								" returned invalid double: " + payload));
		}
		return true;
	}

	void setCommandIs(const string &cmd_mssg, const string &data_mssg) {
		const std::string operation = operationForKey("Redis SET", cmd_mssg);
		ensureConnected(operation);
		reply_ = (redisReply *)redisCommand(context_, "SET %s %s", cmd_mssg.c_str(), data_mssg.c_str());
		if (NULL == reply_) {
			throw(runtime_error(operation + " failed: " + redisContextError(context_)));
		}
		if (REDIS_REPLY_ERROR == reply_->type) {
			const std::string error = redisReplySummary(reply_);
			freeCurrentReply();
			throw(runtime_error(operation + " failed: " + error));
		}
		freeCurrentReply();
	}

	void setCommandBatch(const std::vector<string> &cmd_mssg_vec, const std::vector<std::array<double, 7>> &data_mssg_vec, const int &n_messages)
	{
		ensureConnected("Redis setCommandBatch");
		string data_mssg_indiv;
		string batch_mssg = "";

		if(cmd_mssg_vec.size() != n_messages || data_mssg_vec.size() != n_messages)
		{
			throw(runtime_error("Not the same number of messages and keys as the expected one in setCommandBatch\n"));
		}

		// set the commands
		int cmd = 0;
		for(int i=0; i<n_messages; i++)
		{
			hDoubleArraytoStringArrayJSON(data_mssg_vec[i], 7, data_mssg_indiv);
			// batch_mssg += "SET " + cmd_mssg_vec[i] + " " + data_mssg_indiv + "\r\n";
			if (redisAppendCommand(context_,"SET %s %s", cmd_mssg_vec[i].c_str(), data_mssg_indiv.c_str()) != REDIS_OK) {
				throw(runtime_error(operationForKey("Redis setCommandBatch append SET",
												   cmd_mssg_vec[i]) +
									" failed: " + redisContextError(context_)));
			}
			++cmd;
		}
	    /* Read (and ignore) the replies */
		while ( cmd-- > 0 )
		{
			int r = redisGetReply(context_, (void **) &reply_ );
			checkRedisGetReplyStatus(r, context_, "Redis setCommandBatch reply");
			try {
				checkRedisReply(reply_, context_, "Redis setCommandBatch reply");
			} catch (...) {
				freeCurrentReply();
				throw;
			}
			freeCurrentReply();
		}
	}

	// Updated for all redis commands involved in Rizon 4S robots
	void setGetBatchRizon4S(const std::vector<string> &cmd_mssg_vec, 
			std::array<double, 7> &get_data_mssg_cmd_torques,
			std::array<double, 3> &get_data_mssg_gripper_params,
			const Eigen::MatrixXd &set_data_mssg_massmatrix,
			const std::vector<std::array<double, 7>> &set_joint_data_mssg_vec, 
			const std::vector<std::array<double, 1>> &set_gripper_status_mssg_vec,
		 	const std::vector<std::array<double, 3>> &set_wrist_data_mssg_vec)
	{
		ensureConnected("Redis setGetBatchRizon4S");
		string data_mssg_indiv;
		string batch_mssg = "";
		int num_joint_data_msgs = set_joint_data_mssg_vec.size();
		int num_wrist_data_msgs = set_wrist_data_mssg_vec.size();
		int num_gripper_status_msgs = set_gripper_status_mssg_vec.size();

		int n_messages = num_joint_data_msgs + num_wrist_data_msgs + num_gripper_status_msgs + 3;

		if(cmd_mssg_vec.size() != n_messages)
		{
			throw(runtime_error("Not the same number of messages and keys as the expected one in setCommandBatch\n"));
		}

		// add get command for commanded joint torques from controller
		int cmd = 0;
		if (redisAppendCommand(context_,"GET %s", cmd_mssg_vec[0].c_str()) != REDIS_OK) {
			throw(runtime_error(operationForKey("Redis setGetBatchRizon4S append GET",
											   cmd_mssg_vec[0]) +
								" failed: " + redisContextError(context_)));
		}
		++cmd;

		// add get command for commanded gripper parameters from controller
		if (redisAppendCommand(context_,"GET %s", cmd_mssg_vec[1].c_str()) != REDIS_OK) {
			throw(runtime_error(operationForKey("Redis setGetBatchRizon4S append GET",
											   cmd_mssg_vec[1]) +
								" failed: " + redisContextError(context_)));
		}
		++cmd;

		// create set commands for robot-provided mass matrix
		hEigentoStringArrayJSON(set_data_mssg_massmatrix, data_mssg_indiv);
		if (redisAppendCommand(context_,"SET %s %s", cmd_mssg_vec[2].c_str(), data_mssg_indiv.c_str()) != REDIS_OK) {
			throw(runtime_error(operationForKey("Redis setGetBatchRizon4S append SET",
											   cmd_mssg_vec[2]) +
								" failed: " + redisContextError(context_)));
		}
		++cmd;

		// create set commands for joint data from robot
		for(int i=0; i < num_joint_data_msgs; i++)
		{
			hDoubleArraytoStringArrayJSON(set_joint_data_mssg_vec[i], 7, data_mssg_indiv);
			if (redisAppendCommand(context_,"SET %s %s", cmd_mssg_vec[i+3].c_str(), data_mssg_indiv.c_str()) != REDIS_OK) {
				throw(runtime_error(operationForKey("Redis setGetBatchRizon4S append SET",
												   cmd_mssg_vec[i+3]) +
									" failed: " + redisContextError(context_)));
			}
			++cmd;
		}
		
		// create set commands for gripper status data from robot
		for(int i=0; i < num_gripper_status_msgs; i++)
		{
			hDoubleArraytoStringArrayJSON(set_gripper_status_mssg_vec[i], 1, data_mssg_indiv);
			if (redisAppendCommand(context_,"SET %s %s", cmd_mssg_vec[i+3+num_joint_data_msgs].c_str(), data_mssg_indiv.c_str()) != REDIS_OK) {
				throw(runtime_error(operationForKey("Redis setGetBatchRizon4S append SET",
												   cmd_mssg_vec[i+3+num_joint_data_msgs]) +
									" failed: " + redisContextError(context_)));
			}
			++cmd;
		}

		// create set commands for wrist ft data from robot
		for(int i=0; i < num_wrist_data_msgs; i++)
		{
			hDoubleArraytoStringArrayJSON(set_wrist_data_mssg_vec[i], 3, data_mssg_indiv);
			if (redisAppendCommand(context_,"SET %s %s", cmd_mssg_vec[i+3+num_joint_data_msgs+num_gripper_status_msgs].c_str(), data_mssg_indiv.c_str()) != REDIS_OK) {
				throw(runtime_error(operationForKey("Redis setGetBatchRizon4S append SET",
												   cmd_mssg_vec[i+3+num_joint_data_msgs+num_gripper_status_msgs]) +
									" failed: " + redisContextError(context_)));
			}
			++cmd;
		}

	    /* Read (and process) the replies to the get commands for joint commands torque and gripper parameters*/
		for(int j=0; j<2; j++)
		{
			int r = redisGetReply(context_, (void **) &reply_ );
			checkRedisGetReplyStatus(r, context_, "Redis setGetBatchRizon4S GET reply");
			try {
				checkRedisReply(reply_, context_, "Redis setGetBatchRizon4S GET reply");
			} catch (...) {
				freeCurrentReply();
				throw;
			}
			bool parse_success = true;
			std::string payload;
			if(j==0) {
				if(reply_->type != REDIS_REPLY_NIL) {
					if(reply_->type != REDIS_REPLY_STRING) {
						parse_success = false;
					} else {
						payload.assign(reply_->str, reply_->len);
						parse_success = hDoubleArrayFromStringArrayJSON(
							get_data_mssg_cmd_torques, 7, payload);
					}
				}
			} else if(j==1) {
				if(reply_->type != REDIS_REPLY_NIL) {
					if(reply_->type != REDIS_REPLY_STRING) {
						parse_success = false;
					} else {
						payload.assign(reply_->str, reply_->len);
						parse_success = hDoubleArrayFromStringArrayJSON(
							get_data_mssg_gripper_params, 3, payload);
					}
				}
			}
			freeCurrentReply();
			if(!parse_success) {
				throw(runtime_error(operationForKey(
					"Redis setGetBatchRizon4S GET reply", cmd_mssg_vec[j]) +
					" returned invalid command array data: " + payload));
			}
			cmd--;
		}

	    /* Read (and ignore) the replies to the set commands */
		while ( cmd-- > 0 )
		{
			int r = redisGetReply(context_, (void **) &reply_ );
			checkRedisGetReplyStatus(r, context_, "Redis setGetBatchRizon4S SET reply");
			try {
				checkRedisReply(reply_, context_, "Redis setGetBatchRizon4S SET reply");
			} catch (...) {
				freeCurrentReply();
				throw;
			}
			freeCurrentReply();
		}
	}

	// write raw eigen vector
	template<typename Derived>
	void setEigenMatrixDerived(const string &cmd_mssg, const Eigen::MatrixBase<Derived> &set_mat) {
		string data_mssg;
		// serialize
		hEigentoStringArrayJSON(set_mat, data_mssg); //this never fails
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
	}

    // write raw eigen vector, but in custom string format
    template<typename Derived>
    void setEigenMatrixDerivedString(const string &cmd_mssg, const Eigen::MatrixBase<Derived> &set_mat) {
    	string data_mssg;
		// serialize
		hEigenToStringArrayCustom(set_mat, data_mssg);
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
    }

	// read raw eigen vector:
	template<typename Derived>
	void getEigenMatrixDerived(const string &cmd_mssg, Eigen::MatrixBase<Derived> &ret_mat) {
		auto success = getCommandIs(cmd_mssg);
		if (!success) {
			freeCurrentReply();
			return;
		}
		const std::string payload(reply_->str, reply_->len);
		freeCurrentReply();
		if(!hEigenFromStringArrayJSON(ret_mat, payload)) {
			throw(runtime_error(operationForKey("Redis GET", cmd_mssg) +
								" returned invalid JSON eigen data: " +
								payload));
		}
	}

	// read raw eigen vector, but from a custom string rather than from json
	template<typename Derived>
	void getEigenMatrixDerivedString(const string &cmd_mssg, Eigen::MatrixBase<Derived> &ret_mat) {
		auto success = getCommandIs(cmd_mssg);
		if (!success) {
			freeCurrentReply();
			return;
		}
		const std::string payload(reply_->str, reply_->len);
		freeCurrentReply();
		if(!hEigenFromStringArrayCustom(ret_mat, payload)) {
			throw(runtime_error(operationForKey("Redis GET", cmd_mssg) +
								" returned invalid custom eigen data: " +
								payload));
		}
	}

	// write raw double array of length n
	template<int n>
	void setDoubleArray(const string &cmd_mssg, const double (&set_array)[n], const int &array_length) {
		validateArrayLength<n>(array_length, cmd_mssg);
		string data_mssg;
		// serialize
		hDoubleArraytoStringArrayJSON(set_array, array_length, data_mssg); //this never fails
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
	}

	// read double array of length n
	template<int n>
	void getDoubleArray(const string &cmd_mssg, double (&ret_array)[n], const int &array_length) {
		validateArrayLength<n>(array_length, cmd_mssg);
		auto success = getCommandIs(cmd_mssg);
		if (!success) {
			freeCurrentReply();
			return;
		}
		const std::string payload(reply_->str, reply_->len);
		freeCurrentReply();
		if(!hDoubleArrayFromStringArrayJSON(ret_array, array_length, payload)) {
			throw(runtime_error(operationForKey("Redis GET", cmd_mssg) +
								" returned invalid JSON double array data: " +
								payload));
		}
	}

	// write raw double array of length n
	template<long unsigned int n>
	void setDoubleArray(const string &cmd_mssg, const std::array<double, n> &set_array, const int &array_length) {
		validateArrayLength<n>(array_length, cmd_mssg);
		string data_mssg;
		// serialize
		hDoubleArraytoStringArrayJSON(set_array, array_length, data_mssg); //this never fails
		// set to server
		setCommandIs(cmd_mssg, data_mssg);
	}

	// read double array of length n
	template<long unsigned int n>
	void getDoubleArray(const string &cmd_mssg, std::array<double, n> &ret_array, const int &array_length) {
		validateArrayLength<n>(array_length, cmd_mssg);
		auto success = getCommandIs(cmd_mssg);
		if (!success) {
			freeCurrentReply();
			return;
		}
		const std::string payload(reply_->str, reply_->len);
		freeCurrentReply();
		if(!hDoubleArrayFromStringArrayJSON(ret_array, array_length, payload)) {
			throw(runtime_error(operationForKey("Redis GET", cmd_mssg) +
								" returned invalid JSON double array data: " +
								payload));
		}
	}

public: // server connectivity tools
	void ping() {
		ensureConnected("Redis PING");
		// PING server to make sure things are working..
        reply_ = (redisReply *)redisCommand(context_,"PING");
        if (NULL == reply_) {
            throw(runtime_error("Redis PING failed: " + redisContextError(context_)));
        }
        if (REDIS_REPLY_ERROR == reply_->type) {
            const std::string error = redisReplySummary(reply_);
            freeCurrentReply();
            throw(runtime_error("Redis PING failed: " + error));
        }
        cout<<"\n\nDriver Redis Task : Pinged Redis server. Reply is, "<<reply_->str<<"\n";
        freeCurrentReply();
	}

public:
	HiredisServerInfo server_info_;
	redisContext *context_;
    redisReply *reply_;
	//TODO: decide if needed. Currently, we throw up if server disconnects
	//bool connected_;
protected:
	template<typename Derived>
	bool hEigentoStringArrayJSON(const Eigen::MatrixBase<Derived> &, std::string &);

	template<int n>
	bool hDoubleArraytoStringArrayJSON(const double (&)[n], const int &, std::string &);

	template<long unsigned int n>
	bool hDoubleArraytoStringArrayJSON(const std::array<double, n> &, const int &, std::string &);
	
	template<typename Derived>
	bool hEigenFromStringArrayJSON(Eigen::MatrixBase<Derived> &, const std::string &);

	template<int n>
	bool hDoubleArrayFromStringArrayJSON(double (&)[n], const int &, const std::string &);

	template<long unsigned int n>
	bool hDoubleArrayFromStringArrayJSON(std::array<double, n> &, const int &, const std::string &);

	template<typename Derived>
	bool hEigenToStringArrayCustom(const Eigen::MatrixBase<Derived> &, std::string &);

	template<typename Derived>
	bool hEigenFromStringArrayCustom(Eigen::MatrixBase<Derived> &, const std::string &);

private:
	void ensureConnected(const std::string &operation) const {
		if (context_ == NULL) {
			throw runtime_error(operation + " failed: Redis context is null");
		}
		if (context_->err != 0) {
			throw runtime_error(operation + " failed: " +
								redisContextError(context_));
		}
	}

	void freeCurrentReply() {
		if (reply_ != NULL) {
			freeReplyObject((void*)reply_);
			reply_ = NULL;
		}
	}

	static std::string operationForKey(const std::string &operation,
									   const std::string &key) {
		return operation + " [" + key + "]";
	}

	template<size_t n>
	static void validateArrayLength(const int &array_length,
									const std::string &key) {
		if (array_length < 0 || static_cast<size_t>(array_length) > n) {
			std::stringstream ss;
			ss << "Invalid Redis array length " << array_length
			   << " for key [" << key << "], storage size is " << n;
			throw runtime_error(ss.str());
		}
	}

	// internal function. prepends robot name to command
	static inline string robotCmd(string robot_name, string cmd) {
		return robot_name + ":" + cmd;
	}
};

//Implementation must be part of header for compile time template specialization
template<typename Derived>
bool CDatabaseRedisClient::hEigentoStringArrayJSON(const Eigen::MatrixBase<Derived>& x, std::string& arg_str)
{
	std::stringstream ss;
	bool row_major = true;
	if(x.cols() == 1) row_major = false; //This is a Vector!
	arg_str = "[";
	if(row_major)
	{// [1 2 3; 4 5 6] == [ [1, 2, 3], [4, 5, 6] ]
	  for(int i=0;i<x.rows();++i){
	    if(x.rows() > 1){
	      // If it is only one row, don't need the second one
	      if(i>0) arg_str.append(",[");
	      else arg_str.append("[");
	    }
	    else if(i>0) arg_str.append(",");
	    for(int j=0;j<x.cols();++j){
	      ss << std::setprecision(12) << x(i,j);
	      if(j>0) arg_str.append(",");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	    if(x.rows() > 1){
	      // If it is only one row, don't need the second one
	      arg_str.append("]");
	    }
	  }
	  arg_str.append("]");
	}
	else
	{// [1 2 3; 4 5 6] == 1 4 2 5 3 6
	  for(int j=0;j<x.cols();++j){
	    if(x.cols() > 1){
	      // If it is only one row, don't need the second one
	      if(j>0) arg_str.append(",[");
	      else arg_str.append("[");
	    }
	    else if(j>0) arg_str.append(",");
	    for(int i=0;i<x.rows();++i){
	      ss << std::setprecision(12) << x(i,j);
	      if(i>0) arg_str.append(",");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	    if(x.cols() > 1){
	      // If it is only one row, don't need the second one
	      arg_str.append("]");
	    }
	  }
	  arg_str.append("]");
	}
	return true;
}

template<int n>
bool CDatabaseRedisClient::hDoubleArraytoStringArrayJSON(const double (&x)[n], const int& array_length, std::string& arg_str)
{
	if(array_length < 0 || array_length > n) return false;
	std::stringstream ss;
	arg_str = "[";
	for(int i=0; i<array_length; ++i)
	{
		if(i>0)
		{
			arg_str.append(",");
		}
		ss << std::setprecision(12) << x[i];
		arg_str.append(ss.str());
		ss.str(std::string());
	}
	arg_str.append("]");
	return true;
}

template<long unsigned int n>
bool CDatabaseRedisClient::hDoubleArraytoStringArrayJSON(const std::array<double, n> &x, const int& array_length, std::string& arg_str)
{
	if(array_length < 0 || static_cast<size_t>(array_length) > n) return false;
	std::stringstream ss;
	arg_str = "[";
	for(int i=0; i<array_length; ++i)
	{
		if(i>0)
		{
			arg_str.append(",");
		}
		ss << std::setprecision(12) << x[i];
		arg_str.append(ss.str());
		ss.str(std::string());
	}
	arg_str.append("]");
	return true;
}

template<typename Derived>
bool CDatabaseRedisClient::hEigenFromStringArrayJSON(Eigen::MatrixBase<Derived>& x, const std::string &arg_str)
{
	Json::Value jval;
    Json::Reader json_reader;
    if(!json_reader.parse(arg_str,jval))
    { return false; }

	if(!jval.isArray()) return false; //Must be an array..
	unsigned int nrows = jval.size();
	if(nrows < 1) return false; //Must have elements.

	bool is_matrix = jval[0].isArray();
	if(!is_matrix)
	{
	  if (Derived::RowsAtCompileTime != Eigen::Dynamic &&
		  Derived::RowsAtCompileTime != static_cast<int>(nrows)) {
		  return false;
	  }
	  if (Derived::ColsAtCompileTime != Eigen::Dynamic &&
		  Derived::ColsAtCompileTime != 1) {
		  return false;
	  }
	  x.derived().resize(nrows, 1);
	  for(unsigned int i=0;i<nrows;++i) {
		if(!jval[i].isNumeric()) return false;
		const double value = jval[i].asDouble();
		if(!std::isfinite(value)) return false;
		x(i,0) = value;
	  }
	}
	else
	{
	  unsigned int ncols = jval[0].size();
	  if(ncols < 1) return false; //Must have elements.
	  if (Derived::RowsAtCompileTime != Eigen::Dynamic &&
		  Derived::RowsAtCompileTime != static_cast<int>(nrows)) {
		  return false;
	  }
	  if (Derived::ColsAtCompileTime != Eigen::Dynamic &&
		  Derived::ColsAtCompileTime != static_cast<int>(ncols)) {
		  return false;
	  }
	  x.derived().resize(nrows,ncols);
	  for(int i=0;i<nrows;++i){
	    if(ncols != jval[i].size()) return false;
	    for(int j=0;j<ncols;++j) {
		  if(!jval[i][j].isNumeric()) return false;
		  const double value = jval[i][j].asDouble();
		  if(!std::isfinite(value)) return false;
	      x(i,j) = value;
		}
	  }
	}
	return true;
}

template<long unsigned int n>
bool CDatabaseRedisClient::hDoubleArrayFromStringArrayJSON(std::array<double, n> &x, const int& array_length, const std::string &arg_str)
{
	if(array_length < 0 || static_cast<size_t>(array_length) > n)
	{
		std::cout << "WARNING : Requested double array size is outside storage bounds\n";
		return false;
	}

	Json::Value jval;
	Json::Reader json_reader;
	if(json_reader.parse(arg_str,jval) && jval.isArray())
	{
		unsigned int nrows = jval.size();
		if(nrows < 1) return false; //Must have elements.

		bool is_matrix = jval[0].isArray();
		if(!is_matrix)
		{
			if(nrows != static_cast<unsigned int>(array_length))
			{
				std::cout << "WARNING : JSON array size inconsistent with double array size\n";
				return false;
			}
			for(int i=0; i<array_length; ++i)
			{
				if(!jval[i].isNumeric()) return false;
				const double value = jval[i].asDouble();
				if(!std::isfinite(value)) return false;
				x[i] = value;
			}
			return true;
		}
		else
		{
			std::cout << "WARNING : Trying to read a JSON Matrix to a double array\n";
			return false;
		}
	}

	std::string custom_str = arg_str;
	std::replace(custom_str.begin(), custom_str.end(), ',', ' ');
	std::replace(custom_str.begin(), custom_str.end(), ';', ' ');
	std::replace(custom_str.begin(), custom_str.end(), '[', ' ');
	std::replace(custom_str.begin(), custom_str.end(), ']', ' ');

	std::stringstream ss(custom_str);
	for(int i=0; i<array_length; ++i)
	{
		if(!(ss >> x[i]) || !std::isfinite(x[i]))
		{
			std::cout << "WARNING : Could not parse double array string\n";
			return false;
		}
	}

	std::string extra_value;
	if(ss >> extra_value)
	{
		std::cout << "WARNING : Double array string has extra values\n";
		return false;
	}

	return true;
}

template<int n>
bool CDatabaseRedisClient::hDoubleArrayFromStringArrayJSON(double (&x)[n], const int& array_length, const std::string &arg_str)
{
	Json::Value jval;
	Json::Reader json_reader;
	if(!json_reader.parse(arg_str,jval))
		{ return false; }

	if(!jval.isArray()) return false; //Must be an array..
	unsigned int nrows = jval.size();
	if(nrows < 1) return false; //Must have elements.

	bool is_matrix = jval[0].isArray();
	if(!is_matrix)
	{
		if(array_length < 0 || array_length > n ||
		   nrows != static_cast<unsigned int>(array_length))
		{
			std::cout << "WARNING : JSON array size inconsistent with double array size\n";
			return false;
		}
		for(int i=0; i<array_length; ++i)
		{
			if(!jval[i].isNumeric()) return false;
			const double value = jval[i].asDouble();
			if(!std::isfinite(value)) return false;
			x[i] = value;
		}
	}
	else
	{
		std::cout << "WARNING : Trying to read a JSON Matrix to a double array\n";
		return false;
	}
	return true;
}

template<typename Derived>
bool CDatabaseRedisClient::hEigenToStringArrayCustom(const Eigen::MatrixBase<Derived>& x, std::string& arg_str)
{
	std::stringstream ss;
	bool row_major = true;
	if(x.cols() == 1) row_major = false; //This is a Vector!
	arg_str = "";
	if(row_major)
	{// [1 2 3; 4 5 6] == '1 2 3; 4 5 6' without the ''
	  for(int i=0;i<x.rows();++i){
	    if(i>0) { arg_str.append("; "); }
	    for(int j=0;j<x.cols();++j){
	      ss << std::setprecision(12) << std::fixed << x(i,j);
	      if(j>0) arg_str.append(" ");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	  }
	}
	else
	{// [1 2 3; 4 5 6] == 1 4; 2 5; 3 6
	  for(int j=0;j<x.cols();++j){
	    if(j>0) arg_str.append("; ");
	    for(int i=0;i<x.rows();++i){
	      ss << std::setprecision(12) << std::fixed << x(i,j);
	      if(i>0) arg_str.append(" ");
	      arg_str.append(ss.str());
	      ss.str(std::string());
	    }
	  }
	}
	return true;
}

template<typename Derived>
bool CDatabaseRedisClient::hEigenFromStringArrayCustom(Eigen::MatrixBase<Derived>& x, const std::string &arg_str)
{
	std::vector<std::vector<double>> rows;
	std::stringstream row_stream(arg_str);
	std::string row_str;
	while (std::getline(row_stream, row_str, ';')) {
		std::stringstream value_stream(row_str);
		std::string token;
		std::vector<double> row;
		while (value_stream >> token) {
			double value = 0;
			if (!parseFiniteDoubleToken(token, value)) {
				return false;
			}
			row.push_back(value);
		}
		if (!row.empty()) {
			rows.push_back(row);
		}
	}

	if(rows.empty()) return false;

	const unsigned int nrows = rows.size();
	const unsigned int ncols = rows[0].size();
	if(ncols < 1) return false;
	for(const auto &row : rows) {
		if(row.size() != ncols) return false;
	}

	bool is_matrix = (nrows > 1);
	if(!is_matrix)
	{
	  if (Derived::RowsAtCompileTime != Eigen::Dynamic &&
		  Derived::RowsAtCompileTime != static_cast<int>(ncols)) {
		  return false;
	  }
	  if (Derived::ColsAtCompileTime != Eigen::Dynamic &&
		  Derived::ColsAtCompileTime != 1) {
		  return false;
	  }
	  x.derived().resize(ncols,1);
	  for(unsigned int i=0;i<ncols;++i) {
		x(i,0) = rows[0][i];
	  }
	}
	else
	{
	  if (Derived::RowsAtCompileTime != Eigen::Dynamic &&
		  Derived::RowsAtCompileTime != static_cast<int>(nrows)) {
		  return false;
	  }
	  if (Derived::ColsAtCompileTime != Eigen::Dynamic &&
		  Derived::ColsAtCompileTime != static_cast<int>(ncols)) {
		  return false;
	  }
	  x.derived().resize(nrows,ncols);
	  for(unsigned int i=0;i<nrows;++i){
	    for(unsigned int j=0;j<ncols;++j) {
			x(i,j) = rows[i][j];
		}
	  }
	}
	return true;
}

} // namespace Flexiv
} // namespace Sai

#endif //SAI_FLEXIV_REDIS_CLIENT_LOCAL_H
