/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef GMAPPING_TASK_TASK_HPP
#define GMAPPING_TASK_TASK_HPP

#include "gmapping/TaskBase.hpp"
#include <gmapping/gmappingTypes.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <boost/thread.hpp>


#include <gmapping/gridfastslam/gridslamprocessor.h>
#include <gmapping/sensor/sensor_base/sensor.h>

namespace gmapping {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * import_types_from "gmapping/CustomType.hpp"
 If this project uses data types that are defined in other oroGen projects,
 these projects should be imported there as well.
 import_types_from "base"
 Declare a new task context (i.e., a component)

 The corresponding C++ class can be edited in tasks/Task.hpp and
 tasks/Task.cpp, and will be put in the gmapping namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','gmapping::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {

    private:

        base::samples::RigidBodyState base_to_laser_;
        base::samples::RigidBodyState map_to_odom_;
        int laser_count_;
        boost::thread* transform_thread_;

        GMapping::GridSlamProcessor* gsp_;
        GMapping::RangeSensor* gsp_laser_;
        GMapping::OdometrySensor* gsp_odom_;

        double gsp_laser_angle_increment_;
        bool got_first_scan_;
        bool got_map_;
        Config config_;

        boost::mutex map_to_odom_mutex_;
        boost::mutex map_mutex_;
        OccupancyGrid map_;

        unsigned int gsp_laser_beam_count_;

        double computePoseEntropy();
        void laserCallBack(base::samples::LaserScan& scan);
        void tfCallBack(base::samples::RigidBodyState& tf);
        
        void updateMap(const Laser& scan);
        bool getOdomPose(GMapping::OrientedPoint& gmap_pose, const base::Time& t);
        bool initMapper(const Laser& scan, GMapping::OrientedPoint& initialPose);
        bool addScan(const Laser& scan, GMapping::OrientedPoint& gmap_pose);

        bool createImage(OccupancyGrid map);

    friend class TaskBase;
    protected:

        /* 
         */
        virtual ::gmapping::OccupancyGrid getDynamicMap();
        virtual void scanTransformerCallback(const base::Time &ts, const ::base::samples::LaserScan &scan_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "gmapping::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
    ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();


        void publishLoop(double transform_publish_period);

        void publishTransform();
    };
}

#endif

