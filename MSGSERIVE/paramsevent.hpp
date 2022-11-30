#ifndef PARAMSEVENT_H
#define PARAMSEVENT_H

#include <iostream>
#include <string>
#include <functional>
#include <map>
#include "myparams.h"
#include <mutex>
#include "qdebug.h"

using std::cout;
using std::endl;
using std::string;
using std::map;

#include "noncopyable.h"
#include "filehanddle.h"

using ObserverFunc = std::function<void(TotalParams,bool)>;
//using ObserverFunc = std::function<void(TotalParams,int)>;

template <typename Func>
class ParamsEvent: NonCopyable {
public:

    explicit ParamsEvent()
    {
        file_handdle.init(total_params);
    }

    int Connect(Func&& f) {
        return Assign(f);
    }

    int Connect(const Func& f) {
        return Assign(f);
    }

    void Disconnect(int key) {
        m_connections.erase(key);
    }

    template <typename ... Args>
    void Notify(Args&& ... args) {
        for(auto &it : m_connections) {
            auto& func = it.second;
            func(std::forward<Args>(args)...);
        }
    }

    TotalParams& getTotalParams()
    {
        return total_params;
    }

    std::mutex& getMylock()
    {
        return  this->m_lock;
    }

    void saveData()
    {
        if(!file_handdle.writeDataLocal(total_params))
        {
            qCritical()<<"保存文件失败";
        }
        else {
            qDebug()<<"保存文件成功";
        }
    }


private:

    template <typename F>
    int Assign(F&& f) {
        int k = m_observerId++;
        m_connections.emplace(k, std::forward<F>(f));
        return k;
    }

    int m_observerId = 0;
    map<int, Func> m_connections;
    TotalParams total_params;
    std::mutex m_lock;
    FileHanddle file_handdle;
};

#endif

