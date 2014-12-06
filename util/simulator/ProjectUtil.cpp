#include <iostream>
#include <rtm/Manager.h>
#include <rtm/RTObject.h>
#include <rtm/DataFlowComponentBase.h>
#include "ProjectUtil.h"

void initRTS(Project &prj, std::vector<ClockReceiver>& receivers)
{
    RTC::Manager& manager = RTC::Manager::instance();

    RTSItem& rts = prj.RTS();
    // load factories
    for (std::map<std::string, RTSItem::rtc>::iterator it 
             = rts.components.begin(); it != rts.components.end(); it++){
        std::string path = it->second.path;
        if (path == "") continue;
#ifdef __APPLE__
        path += ".dylib";
#else
        path += ".so";
#endif
        std::cout << "loading " << path << std::endl; 
        std::string initfunc = it->second.name + "Init";
        manager.load(path.c_str(), initfunc.c_str());
    }
    // create components
    for (std::map<std::string, RTSItem::rtc>::iterator it 
             = rts.components.begin(); it != rts.components.end(); it++){
        RTC::RTObject_impl *rtc = manager.getComponent(it->first.c_str());
        if (!rtc){
            if (it->second.name == ""){
                std::cerr << "factory name for " << it->first << " is not defined" << std::endl;
                continue;
            }                
            std::cout << "creating " << it->first << std::endl;
            std::string args = it->second.name + "?instance_name=" + it->first;
            rtc = manager.createComponent(args.c_str());
        }
        if (!rtc){
            std::cerr << "failed to create RTC(" << it->first << ")" << std::endl;
            continue;
        }
        RTC::ExecutionContextList_var eclist = rtc->get_owned_contexts();
        for(CORBA::ULong i=0; i < eclist->length(); ++i){
            if(!CORBA::is_nil(eclist[i])){
                OpenRTM::ExtTrigExecutionContextService_var execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
                if(!CORBA::is_nil(execContext)){
                    std::cout << it->first << ":" << it->second.period << std::endl;
                    receivers.push_back(ClockReceiver(execContext, it->second.period));
                    execContext->activate_component(rtc->getObjRef());
                }
            }
        }
        // set configuration
        {
            RTC::RTObject_var rtc = findRTC(it->first);
            for (size_t i=0; i<it->second.configuration.size(); i++){
                setConfiguration(rtc, it->second.configuration[i].first,
                                 it->second.configuration[i].second);
            }
        }
    }
    // make connections
    for (std::vector<std::pair<std::string, std::string> >::iterator it
             = rts.connections.begin(); it != rts.connections.end(); it++){
        std::cout << "making a connection between "
                  << it->first << " and " << it->second << std::endl;
        int pos1 = it->first.find('.');
        std::string comp1 = it->first.substr(0, pos1);
        std::string port1 = it->first;
        int pos2 = it->second.find('.');
        std::string comp2 = it->second.substr(0, pos2);
        std::string port2 = it->second;

        RTC::RTObject_var rtc1 = findRTC(comp1);
        if (!rtc1){
            std::cerr << "can't find a component named " << comp1 << std::endl;
            return;
        }
        RTC::RTObject_var rtc2 = findRTC(comp2);
        if (!rtc2){
            std::cerr << "can't find a component named " << comp2 << std::endl;
            return;
        }
        RTC::PortServiceList_var ports1 = rtc1->get_ports();
        RTC::PortServiceList_var ports2 = rtc2->get_ports();

        RTC::PortService_ptr portObj1=NULL, portObj2=NULL; 
        for(CORBA::ULong i = 0; i < ports1->length(); ++i ){
            RTC::PortProfile_var profile = ports1[i]->get_port_profile();
            std::string portName(profile->name);
            if (portName == port1){
                portObj1 = ports1[i];
                break;
            }
        }
        if (!portObj1) {
            std::cerr << "can't find a port named " << port1 << std::endl;
            return; 
        }
        for(CORBA::ULong i = 0; i < ports2->length(); ++i ){
            RTC::PortProfile_var profile = ports2[i]->get_port_profile();
            std::string portName(profile->name);
            if (portName == port2){
                portObj2 = ports2[i];
                break;
            }
        }
        if (!portObj2) {
            std::cerr << "can't find a port named " << port2 << std::endl;
            return;
        }
        connectPorts(portObj1, portObj2);
    }
}
