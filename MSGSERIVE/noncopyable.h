#ifndef NONCOPYABLE_H
#define NONCOPYABLE_H

class NonCopyable {
    public:
        //禁用复制构造函数
        NonCopyable(const NonCopyable& n) = delete; // deleted
        //禁用赋值构造函数
        NonCopyable& operator=(const NonCopyable& n) = delete; // deleted
        NonCopyable() = default; // available
};

#endif // NONCOPYABLE_H
