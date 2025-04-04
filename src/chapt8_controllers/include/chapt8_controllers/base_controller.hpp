#ifndef BASE_CONTROLLER_HPP
#define BASE_CONTROLLER_HPP

namespace controllers
{
    class BaseController
    {
    public:
        virtual void start() = 0;
        virtual void stop() = 0;
        virtual ~BaseController() {}
    };
} // namespace controllers

#endif // BASE_CONTROLLER_HPP