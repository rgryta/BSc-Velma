/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "lwr_gazebo.h"
#include <rtt/Logger.hpp>
#include "velma_sim_conversion.h"
#include <gazebo/physics/dart/DARTModel.hh>
#include <gazebo/physics/dart/DARTJoint.hh>
#include <iostream>

//#define CALCULATE_TOOL_INERTIA

#ifdef CALCULATE_TOOL_INERTIA
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/rigidbodyinertia.hpp>
#endif

using namespace RTT;
using std::cout;
using std::endl;

void LWRGazebo::getExternalForces(LWRGazebo::Joints &q) {
    for (int i=0; i<joints_.size(); i++) {
        q[i] = joints_[i]->GetForce(0);
    }
}

void LWRGazebo::getJointPositionAndVelocity(LWRGazebo::Joints &q, LWRGazebo::Joints &dq) {
    for (int i=0; i<joints_.size(); i++) {
        q[i] = joints_[i]->Position();
        dq[i] = joints_[i]->GetVelocity(0);
    }
}

void LWRGazebo::setForces(const LWRGazebo::Joints &t) {
    for (int i=0; i<joints_.size(); i++) {
        joints_[i]->SetForce(0, t[i]);
    }
}

void LWRGazebo::getGravComp(LWRGazebo::Joints &t) {
    KDL::Vector gr = gz2kdl(gazebo::physics::get_world()->Gravity());

    gazebo::physics::LinkPtr link = links_[6];
    gazebo::physics::JointPtr joint = joints_[6];
    KDL::Frame T_W_L7 = gz2kdl(link->WorldPose());
    KDL::Vector cog = T_W_L7 * KDL::Vector(tool_.com.x, tool_.com.y, tool_.com.z);
    KDL::Vector r = cog - gz2kdl(joint->WorldPose().Pos());
    double mass = tool_.m;
    KDL::Vector torque = r * (mass * gr);
    KDL::Vector axis = gz2kdl(joint->GlobalAxis(0));
    t[6] = KDL::dot(axis, torque);

    for (int i = 6; i > 0; i--) {
        link = links_[i-1];
        joint = joints_[i-1];
        cog = (cog * mass + gz2kdl(link->WorldCoGPose().Pos()) * link->GetInertial()->Mass()) / (mass+link->GetInertial()->Mass());
        mass += link->GetInertial()->Mass();
        r = cog - gz2kdl(joint->WorldPose().Pos());
        torque = r * (mass * gr);
        axis = gz2kdl(joint->GlobalAxis(0));
        t[i-1] = KDL::dot(axis, torque);
    }

    for (int i = 0; i < 7; ++i) {
        t[i] = -t[i];
    }
}

bool LWRGazebo::gazeboConfigureHook(gazebo::physics::ModelPtr model) {
    Logger::In in("LWRGazebo::gazeboConfigureHook");

    if(model.get() == NULL) {
        Logger::log() << Logger::Error << "gazebo model is NULL" << Logger::endl;
        return false;
    }

    model_ = model;

    counter_ = 0;
    return true;
}

void LWRGazebo::setInitialPosition(const std::vector<double> &init_q) {
    // non-RT code
    init_q_vec_ = init_q;

    for (int i = 0; i < joints_.size(); ++i) {
        gazebo::physics::DARTJointPtr joint_dart = boost::dynamic_pointer_cast<gazebo::physics::DARTJoint>(joints_[i]);
        if (joint_dart != NULL) {
            joint_dart->GetDARTJoint()->setPosition(0, init_q_vec_[i]);
        }
        joints_[i]->SetPosition(0, init_q_vec_[i]);
        joints_[i]->SetVelocity(0, 0);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void LWRGazebo::gazeboUpdateHook(gazebo::physics::ModelPtr model)
{
    if (!mm_) {
        return;
    }

#ifdef CALCULATE_TOOL_INERTIA
    //
    // This code calculates and displays inertia of tool.
    // It uses real mass matrix (taken from Gazebo).
    //
    dart::dynamics::SkeletonPtr sk = boost::dynamic_pointer_cast<gazebo::physics::DARTModel >(model)->DARTSkeleton();

    Eigen::MatrixXd mm = sk->getMassMatrix();

    Eigen::Matrix<int, 7, 1 > idx;

    idx(0) = sk->getJoint(name_ + "_arm_0_joint")->getIndexInSkeleton(0);
    idx(1) = sk->getJoint(name_ + "_arm_1_joint")->getIndexInSkeleton(0);
    idx(2) = sk->getJoint(name_ + "_arm_2_joint")->getIndexInSkeleton(0);
    idx(3) = sk->getJoint(name_ + "_arm_3_joint")->getIndexInSkeleton(0);
    idx(4) = sk->getJoint(name_ + "_arm_4_joint")->getIndexInSkeleton(0);
    idx(5) = sk->getJoint(name_ + "_arm_5_joint")->getIndexInSkeleton(0);
    idx(6) = sk->getJoint(name_ + "_arm_6_joint")->getIndexInSkeleton(0);

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            tmp_MassMatrix_out_(i,j) = mm(idx(i),idx(j));
        }
    }

    if (counter_ < 1000) {
        ++counter_;
    }
    else {
        counter_ = 0;

        dart::dynamics::BodyNode *b = sk->getJoint(name_ + "_arm_6_joint")->getChildBodyNode();

        std::vector<dart::dynamics::BodyNode* > all;
        std::vector<dart::dynamics::BodyNode* > to_check;
        std::vector<dart::dynamics::BodyNode* > to_check_new;
        to_check.push_back(b);
        all.push_back(b);

        KDL::Frame base_tf;
        tf::transformEigenToKDL(b->getTransform(), base_tf);

        while (to_check.size() > 0) {
            to_check_new.clear();
            for (int i = 0; i < to_check.size(); ++i) {
                for (int j = 0; j < to_check[i]->getNumChildBodyNodes(); ++j) {
                    b = to_check[i]->getChildBodyNode(j);
                    to_check_new.push_back( b );
                    all.push_back(b);
                }
            }
            to_check = to_check_new;
        }

        cout << endl << name_ << endl;
        // calculate com
        KDL::Vector COM;
        double MASS = 0;
        for (int i = 0; i < all.size(); ++i) {
            KDL::Vector com;
            tf::vectorEigenToKDL(all[i]->getLocalCOM(), com);
            KDL::Frame tf;
            tf::transformEigenToKDL(all[i]->getTransform(), tf);
            COM = COM + (base_tf.Inverse() * tf) * com * all[i]->getMass();
            MASS += all[i]->getMass();
        }
        COM = COM * (1.0 / MASS);
		cout << endl << "          m: " << MASS << endl << "          com:" << endl << "            x: " << COM.x() << endl << "            y: " << COM.y() << endl << "            z: " << COM.z() << endl;
        KDL::Frame T_B_WI = base_tf * KDL::Frame(COM);
        KDL::RigidBodyInertia RBI;
        // calculate moment of inertia
        for (int i = 0; i < all.size(); ++i) {
            double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
            all[i]->getMomentOfInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
            KDL::Vector com;
            tf::vectorEigenToKDL(all[i]->getLocalCOM(), com);
            KDL::RigidBodyInertia rbI(all[i]->getMass(), com,  KDL::RotationalInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz));
            KDL::Frame T_B_L;
            tf::transformEigenToKDL(all[i]->getTransform(), T_B_L);
            KDL::Frame T_B_LI = T_B_L * KDL::Frame(com);
            KDL::RigidBodyInertia rbI_WI = (T_B_WI.Inverse() * T_B_LI) * rbI;
            RBI = RBI + rbI_WI;
        }
		cout << "          ixx: " << RBI.getRotationalInertia().data[0]  << endl << "          ixy: " << RBI.getRotationalInertia().data[1]  << endl << "          ixz: " << RBI.getRotationalInertia().data[2]  << endl;
		cout << "          iyy: " << RBI.getRotationalInertia().data[4]  << endl << "          iyz: " << RBI.getRotationalInertia().data[5]  << endl << "          izz: " << RBI.getRotationalInertia().data[8]  << endl << endl;
    }
#else
    //
    // Use fixed tool inertia.
    //
    // mass matrix
    mm_->updatePoses(model);
    const Eigen::MatrixXd &mm = mm_->getMassMatrix();

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            tmp_MassMatrix_out_(i,j) = mm(i,j);
        }
    }
#endif

    // gravity forces
    Joints grav;
    for (int i = 0; i < 7; ++i) {
        grav[i] = 0;
    }
    getGravComp(grav);

    // gravity forces
    for (int i = 0; i < 7; i++) {
        tmp_GravityTorque_out_[i] = grav[i];
    }

    Joints ext_f;
    getExternalForces(ext_f);

    // external forces
    for (int i = 0; i < 7; i++) {
        tmp_JointTorque_out_[i] = ext_f[i] - grav[i];
    }

    Joints q, dq;
    getJointPositionAndVelocity(q, dq);

    // joint position
    for (int i = 0; i < 7; i++) {
        tmp_JointPosition_out_[i] = q[i];
    }

    // joint velocity
    for (int i = 0; i < 7; i++) {
        tmp_JointVelocity_out_[i] = dq[i];
    }

/*
    // TODO
    // calculate the wrench on the wrist
    Eigen::Isometry3d ET_WO_Wr = right_tool_bn_dart_->getTransform();
    Eigen::Vector3d grav_Wr = ET_WO_Wr.inverse().rotation() * Eigen::Vector3d(0,0,-9.81);
    KDL::Wrench grav_reaction_Wr = KDL::Frame(right_tool_com_W_) * KDL::Wrench(KDL::Vector(grav_Wr(0), grav_Wr(1), grav_Wr(2)), KDL::Vector());
    Eigen::Vector6d wr_Wr = right_tool_bn_dart_->getBodyForce();
    KDL::Wrench wrr_Wr = KDL::Wrench( -KDL::Vector(wr_Wr(3), wr_Wr(4), wr_Wr(5)), -KDL::Vector(wr_Wr(0), wr_Wr(1), wr_Wr(2)) ) - grav_reaction_Wr;
    tmp_r_CartesianWrench_out_.force.x = wrr_Wr.force.x();
    tmp_r_CartesianWrench_out_.force.y = wrr_Wr.force.y();
    tmp_r_CartesianWrench_out_.force.z = wrr_Wr.force.z();
    tmp_r_CartesianWrench_out_.torque.x = wrr_Wr.torque.x();
    tmp_r_CartesianWrench_out_.torque.y = wrr_Wr.torque.y();
    tmp_r_CartesianWrench_out_.torque.z = wrr_Wr.torque.z();

    Eigen::Isometry3d ET_WO_Wl = left_tool_bn_dart_->getTransform();
    Eigen::Vector3d grav_Wl = ET_WO_Wl.inverse().rotation() * Eigen::Vector3d(0,0,-9.81);
    KDL::Wrench grav_reaction_Wl = KDL::Frame(left_tool_com_W_) * KDL::Wrench(KDL::Vector(grav_Wl(0), grav_Wl(1), grav_Wl(2)), KDL::Vector());
    Eigen::Vector6d wr_Wl = left_tool_bn_dart_->getBodyForce();
    KDL::Wrench wrr_Wl = KDL::Wrench( -KDL::Vector(wr_Wl(3), wr_Wl(4), wr_Wl(5)), -KDL::Vector(wr_Wl(0), wr_Wl(1), wr_Wl(2)) ) - grav_reaction_Wl;
    tmp_l_CartesianWrench_out_.force.x = wrr_Wl.force.x();
    tmp_l_CartesianWrench_out_.force.y = wrr_Wl.force.y();
    tmp_l_CartesianWrench_out_.force.z = wrr_Wl.force.z();
    tmp_l_CartesianWrench_out_.torque.x = wrr_Wl.torque.x();
    tmp_l_CartesianWrench_out_.torque.y = wrr_Wl.torque.y();
    tmp_l_CartesianWrench_out_.torque.z = wrr_Wl.torque.z();
*/

    bool tmp_command_mode;

    // exchange the data between Orocos and Gazebo
    {
        RTT::os::MutexLock lock(gazebo_mutex_);

        MassMatrix_out_ = tmp_MassMatrix_out_;
        GravityTorque_out_ = tmp_GravityTorque_out_;
        JointTorque_out_ = tmp_JointTorque_out_;
        JointPosition_out_ = tmp_JointPosition_out_;
        JointVelocity_out_ = tmp_JointVelocity_out_;
        CartesianWrench_out_ = tmp_CartesianWrench_out_;
        tmp_JointTorqueCommand_in_ = JointTorqueCommand_in_;
        tmp_command_mode = command_mode_;
        data_valid_ = true;
    }

    // torque command
    if (tmp_command_mode) {
        for (int i = 0; i < 7; i++) {
            grav[i] += tmp_JointTorqueCommand_in_[i];
        }
    }
    else {
        for (int i = 0; i < joints_.size(); i++) {
            grav[i] += 10.0 * (init_q_vec_[i] - tmp_JointPosition_out_[i]);
        }
    }

    setForces(grav);
}

