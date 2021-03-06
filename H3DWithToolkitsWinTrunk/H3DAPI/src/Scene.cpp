//////////////////////////////////////////////////////////////////////////////
//    Copyright 2004-2014, SenseGraphics AB
//
//    This file is part of H3D API.
//
//    H3D API is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    H3D API is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with H3D API; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    A commercial license is also available. Please contact us at 
//    www.sensegraphics.com for more information.
//
//
/// \file Scene.cpp
/// \brief CPP file for Scene, X3D scene-graph node
///
//
//
//////////////////////////////////////////////////////////////////////////////

#include <H3D/Scene.h>
#include <H3DUtil/TimeStamp.h>
#ifdef HAVE_PROFILER
#include <H3DUtil/H3DTimer.h>
#endif
#include <H3D/DeviceInfo.h>
#include <GL/glew.h>
#ifdef HAVE_GLUT
#ifdef MACOSX
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#ifdef FREEGLUT
#include <GL/freeglut.h>
#endif
#endif // HAVE_GLUT
#include <H3D/PeriodicUpdate.h>
#include <H3D/GLUTWindow.h>
#include <H3D/X3DShapeNode.h>

#include <H3D/X3DGroupingNode.h>
#include <H3D/ProfilesAndComponents.h>
#include <H3D/H3DNavigation.h>
#include <H3D/NavigationInfo.h>
#include <H3D/H3DMultiPassRenderObject.h>
#include <H3D/Anchor.h>
#include <H3D/DirectionalLight.h>
#include <H3D/GlobalSettings.h>
#include <H3D/SAIFunctions.h>
#include <H3D/Shape.h>
#include <H3D/Inline.h>
#include <H3D/H3DSingleTextureNode.h>
#include <H3D/X3DProgrammableShaderObject.h>
#include <H3DUtil/ThreadPool.h>

#ifndef H3D_WINDOWS
#include <unistd.h>
#endif

using namespace H3D;


/////////////////////////////////////////////////////////////////////////////
//
// Initialize static members
//

H3DUtil::MutexLock Scene::callback_lock;
Scene::CallbackList Scene::callbacks;
Scene::ProgramSettingsCallbackList Scene::program_settings_callbacks;
Scene::ProgramSettings Scene::program_settings;
// Add this node to the H3DNodeDatabase system.
H3DNodeDatabase Scene::database( 
                                "Scene", 
                                &(newInstance<Scene>), 
                                typeid( Scene ) );

namespace SceneInternals {
  FIELDDB_ELEMENT( Scene, sceneRoot, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Scene, window, INPUT_OUTPUT )
  FIELDDB_ELEMENT( Scene, frameRate, OUTPUT_ONLY )
  FIELDDB_ELEMENT( Scene, maxFrameRate, INPUT_OUTPUT )
#ifdef HAVE_PROFILER
  FIELDDB_ELEMENT( Scene, profiledResult, INPUT_OUTPUT )
#endif
}


set< Scene* > Scene::scenes;
auto_ptr< SFTime > Scene::time(new SFTime( TimeStamp() ) );
auto_ptr< Scene::EventSink > Scene::eventSink(new EventSink);

//SFTime *Scene::time = new SFTime( TimeStamp() );
//Scene::EventSink *Scene::eventSink = new EventSink;

namespace SceneInternal {

  void idle() {
    try {
      for( set< Scene * >::iterator i = Scene::scenes.begin();
           i != Scene::scenes.end();
           ++i ) {
        if( (*i)->isActive() )
          (*i)->idle();
      }
    }
    catch (const Exception::QuitAPI &) {
#ifdef FREEGLUT
      for (set< Scene * >::iterator i = Scene::scenes.begin(); i != Scene::scenes.end(); ++i) {
        (*i)->setSceneRoot(NULL);
      }
        glutLeaveMainLoop();
#else
      // normal glut does not support exiting main loop. Have to quit 
      // using exit. This will not call any destructors.
      exit(1);
#endif 
    }
    
    catch (const Exception::H3DException &e) {
      Console(LogLevel::Error) << e << endl;
#ifdef FREEGLUT
      for (set< Scene * >::iterator i = Scene::scenes.begin(); i != Scene::scenes.end(); ++i) {
        (*i)->setSceneRoot(NULL);
      }
      glutLeaveMainLoop();
#else
      // normal glut does not support exiting main loop. Have to quit 
      // using exit. This will not call any destructors.
      exit(1);
#endif 
    }
  }
}
#ifdef HAVE_PROFILER
std::string Scene::generateProfileResult()
{
  haptic_result;
  H3D_sofa_result;
  this->H3D_scene_result;
  
  std::stringstream result;

  // Compute exclusive times for each group node
  // i.e. Time for group minus total time reported by children
  const H3DTime update_interval = 1;
  if( time->getValue () - exclusive_times_update_time > update_interval ) {
    exclusive_times_update_time = time->getValue();
    if( X3DGroupingNode* g = dynamic_cast <X3DGroupingNode*> (sceneRoot->getValue()) ) {
      ProfileNodeResultVec exclusive_times;
      computeExclusiveTimes( *g, exclusive_times );

      // Sort descending and trim
      const int max_nodes = 10;
      const H3DTime max_time = 1;

      ProfileNodeResultVec exclusive_times_render;
      exclusive_times_render.reserve( exclusive_times.size() );
      for( ProfileNodeResultVec::iterator i = exclusive_times.begin(); i != exclusive_times.end(); ++i ) {
        const ProfileNodeResult& r = *i;
        if( TimeStamp() - r.render_time_last < max_time ) {
          exclusive_times_render.push_back( r );
        }
      }
      std::sort( exclusive_times_render.begin(), exclusive_times_render.end(), compareRenderTime );
      if( exclusive_times_render.size() > max_nodes ) {
        exclusive_times_render.erase( exclusive_times_render.begin() + max_nodes, exclusive_times_render.end() );
      }

      ProfileNodeResultVec exclusive_times_traverse;
      exclusive_times_traverse.reserve( exclusive_times.size() );
      for( ProfileNodeResultVec::iterator i = exclusive_times.begin(); i != exclusive_times.end(); ++i ) {
        const ProfileNodeResult& r = *i;
        if( TimeStamp() - r.traverse_time_last < max_time ) {
          exclusive_times_traverse.push_back( r );
        }
      }
      std::sort( exclusive_times_traverse.begin(), exclusive_times_traverse.end(), compareTraverseTime );
      if( exclusive_times_traverse.size() > max_nodes ) {
        exclusive_times_traverse.erase( exclusive_times_traverse.begin() + max_nodes, exclusive_times_traverse.end() );
      }

      // Format
      std::stringstream ss;
      ss << "Render:" << std::endl;
      for( ProfileNodeResultVec::iterator i = exclusive_times_render.begin(); i != exclusive_times_render.end(); ++i ) {
        ProfileNodeResult& r = *i;

        std::string child_names;
        if( !r.node->hasName() ) {
          if( X3DGroupingNode* group = dynamic_cast <X3DGroupingNode*> (r.node) ) {
            for( NodeVector::const_iterator j = group->children->begin(); j != group->children->end(); ++j ) {
              if( *j ) {
                child_names += (*j)->getName() + " ";
              }
            }
          }
        }

        ss << std::setprecision( 2 ) << r.render_time*1000.0 << " ms " << r.node->getName() << "(" << r.node->getTypeName() << ") " << child_names << endl;
      }
      ss << std::endl;
      ss << "Traverse:" << std::endl;
      for( ProfileNodeResultVec::iterator i = exclusive_times_traverse.begin(); i != exclusive_times_traverse.end(); ++i ) {
        ProfileNodeResult& r = *i;

        std::string child_names;
        if( !r.node->hasName() ) {
          if( X3DGroupingNode* group = dynamic_cast <X3DGroupingNode*> (r.node) ) {
            for( NodeVector::const_iterator j = group->children->begin(); j != group->children->end(); ++j ) {
              if( *j ) {
                child_names += (*j)->getName() + " ";
              }
            }
          }
        }

        ss << std::setprecision( 2 ) << r.traverse_time*1000.0 << " ms " << r.node->getName() << "(" << r.node->getTypeName() << ") " << child_names << endl;
      }
      exclusive_times_string = ss.str();
    }
  }

  result << "====================================Node Times===================================" << std::endl;
  result << exclusive_times_string;
  result << "=======================================END=======================================" << std::endl;
  
  if(!H3D_scene_result.isEmpty())
  {
    result<<std::endl
          <<"====================================Main Thread=================================="<<std::endl;
    result<<H3D_scene_result.getResult();
    result<<"=======================================END======================================="<<std::endl;
  }
  if(!haptic_result.isEmpty())
  {
    result<<"===================================Haptic Thread================================="<<std::endl;
    result<<haptic_result.getResult();
    result<<"=======================================END======================================="<<std::endl;
  }
  if(!H3D_sofa_result.isEmpty())
  {
    result<<"====================================Sofa Thread=================================="<<std::endl;
    result<<H3D_sofa_result.getResult();
    result<<"=======================================END======================================="<<std::endl;
  }

  return result.str();
}

void Scene::computeExclusiveTimes( Node& _node, ProfileNodeResultVec& _result ) {

  if( X3DGroupingNode* group = dynamic_cast <X3DGroupingNode*> ( &_node ) ) {

    std::pair < H3DTime, H3DTime > times = group->getChildTimes();
    ProfileNodeResult r = { 
      group, group->time_in_last_render - times.first, 
      group->time_in_last_traverseSG - times.second,
      group->time_last_render,
      group->time_last_traverseSG
    };
    _result.push_back( r );

    // Recurse
    for( NodeVector::const_iterator i = group->children->begin(); i != group->children->end(); ++i ) {
      if( *i ) {
        computeExclusiveTimes( **i, _result );
      }
    }

  } else if( Inline* inl = dynamic_cast <Inline*> (&_node) ) {

    // Recurse
    for( NodeVector::const_iterator i = inl->loadedScene->begin(); i != inl->loadedScene->end(); ++i ) {
      if( *i ) {
        computeExclusiveTimes( **i, _result );
      }
    }

  }

  
}

#endif
void Scene::idle() {
  // calculate and set frame rate
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::begin("H3D_scene");
  H3DUtil::H3DTimer::stepBegin("H3D_scene_loop");

#endif // HAVE_PROFILER
  
  H3DFloat max_fr= maxFrameRate->getValue();
  if ( max_fr > 0 ) {
    H3DFloat min_frame_time= 1.0f / max_fr;

    TimeStamp t0;
    TimeStamp dt0= t0 - last_time;

    H3DTime sleep_time= min_frame_time-dt0;
    if ( sleep_time > 0 ) {
#ifdef H3D_WINDOWS
      Sleep ( (DWORD)(sleep_time*1000.0) );
#else
      usleep( sleep_time*1e6 );
#endif
    }
  }

  TimeStamp t;
  TimeStamp dt = t - last_time;
  frameRate->setValue( 1.0f / (H3DFloat)(dt), id );
  last_time = t;
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepBegin("Time_update");
#endif
  time->setValue( t, id );
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepEnd("Time_update");
#endif

#ifdef THREAD_LOCK_DEBUG
#ifndef HAVE_PROFILER
  static TimeStamp last_update_time;
  #define THREAD_LOCK_DEBUG_UPDATE_INTERVAL 2
  ThreadBase::ThreadLockInfo &main_info =ThreadBase::thread_lock_info[ ThreadBase::getMainThreadId() ];
  main_info.total_run_time += dt;
  main_info.period_start_time = t;
  if( t - last_update_time > THREAD_LOCK_DEBUG_UPDATE_INTERVAL ) {
    Console(LogLevel::Error) << "*************************************************" << endl;
    for( ThreadBase::ThreadLockInfoMap::const_iterator i = ThreadBase::thread_lock_info.begin();
         i != ThreadBase::thread_lock_info.end(); ++i ) {
      ThreadBase *thread = H3DUtil::ThreadBase::getThreadById( (*i).first );
      const ThreadBase::ThreadLockInfo &info = (*i).second;
      if( thread ) {
        Console(LogLevel::Error) << "Thread:        " << thread->getThreadName() << endl;
      } else {
        if( pthread_equal( (*i).first, ThreadBase::getMainThreadId() ) ) {
          Console(LogLevel::Error) << "Thread:        " << "Main Thread" << endl;
          static double last_main_thread_lock_time = 0;
          Console(LogLevel::Error) << "Lock (%) inst: " << (info.total_run_time == 0 ? 0 : (info.total_lock_time - last_main_thread_lock_time )/ (t - last_update_time)) *100  << endl;
          last_main_thread_lock_time = info.total_lock_time;

        } else {
          Console(LogLevel::Error) << "Thread:        " << "Non ThreadBase" << endl;
        }
      }
      Console(LogLevel::Error)   << "Tot runtime:   " << info.total_run_time << endl;
      Console(LogLevel::Error)   << "Lock (s):      " << info.total_lock_time << endl;
      Console(LogLevel::Error)   << "Lock (%):      " << (info.total_run_time == 0 ? 0 : (info.total_lock_time / info.total_run_time) *100)  << endl << endl;
    }
    last_update_time = t;
  }
#endif

#endif

#ifdef THREAD_LOCK_DEBUG
#ifdef HAVE_PROFILER
  #define THREAD_LOCK_DEBUG_UPDATE_INTERVAL 2
    static TimeStamp last_update_time;

  ThreadBase::ThreadLockInfo &main_info =ThreadBase::thread_lock_info[ ThreadBase::getMainThreadId() ];
  std::stringstream thread_debug_info_main, thread_debug_info_haptic, thread_debug_info_sofa;
  main_info.total_run_time += dt;
  main_info.period_start_time = t;
  if( t - last_update_time > THREAD_LOCK_DEBUG_UPDATE_INTERVAL ) {
    for( ThreadBase::ThreadLockInfoMap::const_iterator i = ThreadBase::thread_lock_info.begin(); i != ThreadBase::thread_lock_info.end(); ++i ) {
      ThreadBase *thread = H3DUtil::ThreadBase::getThreadById( (*i).first );
      const ThreadBase::ThreadLockInfo &info = (*i).second;
      if( thread ) 
      {
        if(pthread_equal((*i).first,this->H3D_sofa_result.getId()))
        {
          // sofa thread data
          //thread_debug_info_sofa   << "Thread lock debug:" << endl;
          thread_debug_info_sofa   << "Thread:        " << thread->getThreadName() << endl;
          thread_debug_info_sofa   << "Tot runtime:   " << info.total_run_time << endl;
          thread_debug_info_sofa   << "Lock (s):      " << info.total_lock_time << endl;
          thread_debug_info_sofa   << "Lock (%):      " << (info.total_run_time == 0 ? 0 : (info.total_lock_time / info.total_run_time) *100)  << endl << endl; 
          H3D_sofa_result.setThread_debug(thread_debug_info_sofa.str());
        }
        else if(pthread_equal((*i).first,this->haptic_result.getId()))
        {
          // sofa thread data
          thread_debug_info_haptic   << "Thread:        " << thread->getThreadName() << endl;
          thread_debug_info_haptic   << "Tot runtime:   " << info.total_run_time << endl;
          thread_debug_info_haptic   << "Lock (s):      " << info.total_lock_time << endl;
          thread_debug_info_haptic   << "Lock (%):      " << (info.total_run_time == 0 ? 0 : (info.total_lock_time / info.total_run_time) *100)  << endl << endl; 
          haptic_result.setThread_debug(thread_debug_info_haptic.str());
        }
        //// other thread
        //Console(temp_output_level)   << "Thread:        " << thread->getThreadName() << endl;
        //Console(temp_output_level)   << "Tot runtime:   " << info.total_run_time << endl;
        //Console(temp_output_level)   << "Lock (s):      " << info.total_lock_time << endl;
        //Console(temp_output_level)   << "Lock (%):      " << (info.total_run_time == 0 ? 0 : (info.total_lock_time / info.total_run_time) *100)  << endl << endl;
      } 
      else 
      {
        if( pthread_equal( (*i).first, ThreadBase::getMainThreadId() ) ) 
        {
          thread_debug_info_main << "Thread:        " << "Main Thread" << endl;
          static double last_main_thread_lock_time = 0;
          thread_debug_info_main << "Lock (%) inst: " << (info.total_run_time == 0 ? 0 : (info.total_lock_time - last_main_thread_lock_time )/ (t - last_update_time)) *100  << endl;
          last_main_thread_lock_time = info.total_lock_time;
          thread_debug_info_main   << "Tot runtime:   " << info.total_run_time << endl;
          thread_debug_info_main   << "Lock (s):      " << info.total_lock_time << endl;
          thread_debug_info_main   << "Lock (%):      " << (info.total_run_time == 0 ? 0 : (info.total_lock_time / info.total_run_time) *100)  << endl << endl;
          H3D_scene_result.setThread_debug(thread_debug_info_main.str());
        }
      }
      
    }
    last_update_time = t;
  }
#endif
#endif
   DefaultAppearance *def_app = NULL;
  
  GlobalSettings *default_settings = GlobalSettings::getActive();
  if( default_settings ) {
    default_settings->getOptionNode( def_app );
  }

  if( def_app ) {
    Appearance *app = def_app->defaultAppearance->getValue();
    if( app ) {
      //app->displayList->callList();
      if( app->material->getValue() ) {
        X3DShapeNode::disable_lighting_if_no_app = false;
      }
      
      RenderProperties *rp = app->renderProperties->getValue();
      if( rp ) {
        X3DAppearanceNode::setDefaultUsingMultiPassTransparency( 
          rp->multiPassTransparency->getValue() );
      } else {
        X3DAppearanceNode::setDefaultUsingMultiPassTransparency( true );
        }
   }
  }

  H3DMultiPassRenderObject::resetCounters();
  shadow_caster->object->clear();
  shadow_caster->light->clear();
  
  DeviceInfo *di = DeviceInfo::getActive();
  X3DChildNode *scene_root = static_cast< X3DChildNode * >( sceneRoot->getValue() );
  if( di ) {
    vector< H3DHapticsDevice * > hds;
    // update the values for all H3DHapticsDevices
    for( DeviceInfo::MFDevice::const_iterator d = di->device->begin();
         d != di->device->end();
         ++d ) {
      H3DHapticsDevice *hd = static_cast< H3DHapticsDevice * >( *d );
      if( hd->initialized->getValue() ) { 
        hd->preRender();
        hd->updateDeviceValues();
      }
      hds.push_back( hd );
      // add profiled result from haptic 
#ifdef HAVE_PROFILER
      if (hd->getThread())
      {// if haptic device thread exist, gather profiled result collected by the haptic thread.
        this->haptic_result.setResult(hd->getThread()->getThreadId(),static_cast<SFString*>(hd->getField("profiledResult"))->getValueAsString());
      }
      //this->haptic_result.setResult(hd->getThread()->getThreadId(),static_cast<SFString*>(hd->getField("profiledResult"))->getValueAsString());
#endif
      
    }
    // traverse the scene graph to collect the HapticObject instances to render.
    auto_ptr<TraverseInfo> ti(new TraverseInfo(hds));

    ti->setUserData( "ShadowCaster", shadow_caster.get() );

#ifdef HAVE_PROFILER
    H3DUtil::H3DTimer::stepBegin("Scene_traverse");
#endif
    if( scene_root ) {
      scene_root->traverseSG( *ti );
    }
#ifdef HAVE_PROFILER
    H3DUtil::H3DTimer::stepEnd("Scene_traverse");
#endif
    // traverse the stylus of all haptics devices
    if( di ) {
      for( DeviceInfo::MFDevice::const_iterator i = di->device->begin();
           i != di->device->end(); ++i ) {
        H3DHapticsDevice *hd = static_cast< H3DHapticsDevice * >( *i );
        Node *stylus = hd->stylus->getValue();
        if( stylus ) {
          const Vec3f &pos = hd->weightedProxyPosition->getValue();
          const Rotation &rot = hd->trackerOrientation->getValue();
          Matrix4f m(rot);
          m[0][3] = pos.x;
          m[1][3] = pos.y;
          m[2][3] = pos.z;
          (*ti).pushMatrices( m,
                           m.inverse() );
          stylus->traverseSG( *ti );
          (*ti).popMatrices();
        }
      }
    }
      
    // render the HapticShapes and HapticForceEffets in the TraverseInfo 
    // instance on the H3DHapticsDevices.
    unsigned int nr_devices = (unsigned int) ti->getHapticsDevices().size();
    for( unsigned int i = 0; i < nr_devices; ++i ) {
      H3DHapticsDevice *hd = ti->getHapticsDevice( i );
      // nr_of_layers is the biggest of the nr of layers in this traverseinfo
      // and last_traverseinfo.
      unsigned int nr_of_layers = ti->nrLayers();
      if( last_traverseinfo && last_traverseinfo->nrLayers() > nr_of_layers )
        nr_of_layers = last_traverseinfo->nrLayers();
      if( hd->initialized->getValue() ) {
        for( unsigned int l = 0; l < nr_of_layers; ++l ) {
          if( l < ti->nrLayers() ) {
            ti->setCurrentLayer( l );
            hd->renderShapes( ti->getHapticShapes( i ), l );
          } else {
            // This layer no longer exists, set the shapes to an empty vector
            // to remove from haptics loop.
            HapticShapeVector tmp_vector;
            hd->renderShapes( tmp_vector, l );
          }
        }
        hd->renderEffects( ti->getForceEffects( i ) );
        hd->postRender();
      }
    }

    // remove the TraverseInfo instance from the last loop. TraverseInfo 
    // instances must be kept alive until its HapticShapes and 
    // HAPIForceEffects are not rendered anymore, which in this case is
    // one scenegraph loop.
    if( last_traverseinfo )
      delete last_traverseinfo;
    last_traverseinfo = ti.release();


  } else {
    // no HapticDevices exist, but we still have to traverse the scene-graph.
    // Haptics is disabled though to avoid unnecessary calculations.
    auto_ptr<TraverseInfo> ti(new TraverseInfo( vector< H3DHapticsDevice * >()));
    ti->setUserData( "ShadowCaster", shadow_caster.get() );
    ti->disableHaptics();
#ifdef HAVE_PROFILER
    H3DUtil::H3DTimer::stepBegin("Scene_traverse");
#endif
    if( scene_root ) {
      scene_root->traverseSG( *ti );
    }
#ifdef HAVE_PROFILER
    H3DUtil::H3DTimer::stepEnd("Scene_traverse");
#endif
    // remove the TraverseInfo instance from the last loop. TraverseInfo 
    // instances must be kept alive until its HapticShapes and 
    // HAPIForceEffects are not rendered anymore, which in this case is 
    // one scenegraph loop.
    if( last_traverseinfo )
      delete last_traverseinfo;
    last_traverseinfo = ti.release();
  }
  
  // call the callback functions added during callback.
  last_traverseinfo->callPostTraverseCallbacks();

  // add the head light to shadow casting nodes if it is active.
  if( !shadow_caster->object->empty() ) {
    bool head_light = false;
    NavigationInfo *ni = NavigationInfo::getActive();
    if( ni ) {
      head_light = 
        ni->headlightShadows->getValue() && 
        ni->headlight->getValue();
    }

    if( head_light ) {
      X3DViewpointNode *vp = X3DViewpointNode::getActive();
      Vec3f direction = Vec3f( 0, 0, -1 );
      if( vp ) {
        direction = 
          vp->accForwardMatrix->getValue().getRotationPart() * 
          (vp->totalOrientation->getValue() * Vec3f( 0, 0, -1 ));
      }
      auto_ptr<DirectionalLight> light(new DirectionalLight());
      light->direction->setValue( direction );
      shadow_caster->light->push_back( light.release() );
    }
    // default shadow caster doesn't call traverse so call the update function directly
    shadow_caster->updateShadowInformation( *last_traverseinfo, shadow_caster.get() );
  }
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepBegin("Graphic_rendering");
#endif
  // call window's render function
  for( MFWindow::const_iterator w = window->begin(); 
       w != window->end(); ++w ) {
    H3DWindowNode *_window = static_cast< H3DWindowNode * >(*w);
    bool used_mpt = _window->getMultiPassTransparency();
    _window->setMultiPassTransparency(
                       last_traverseinfo->getMultiPassTransparency() );
    if( used_mpt != _window->getMultiPassTransparency() ) {
      H3DDisplayListObject::DisplayList::rebuildAllDisplayLists();
    }
    _window->render( scene_root );
  }
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepEnd("Graphic_rendering");
#endif
  // update the eventSink
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepBegin("Event_sink_update");
#endif
  eventSink->upToDate();
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepEnd("Event_sink_update");
#endif

#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepBegin("Scene_callbacks");
#endif
  callback_lock.lock();
  try {
    // execute callbacks
    for( CallbackList::iterator i = callbacks.begin();
         i != callbacks.end(); ) {
      CallbackCode c = (*i).first( (*i).second );
      if( c == CALLBACK_DONE ) {
        i = callbacks.erase( i );
      } else {
        ++i;
      }
    }
  } catch (const Exception::QuitAPI &) {
    callback_lock.unlock();
    throw Exception::QuitAPI();
  } catch( const Exception::H3DException &exp ) {
    callback_lock.unlock();
    H3DUtil::Console( LogLevel::Error ) << exp.message << std::endl;
  } catch( const std::exception &e ) {
    callback_lock.unlock();
    H3DUtil::Console( LogLevel::Error ) << e.what() << std::endl;
  } catch( ... ) {
    callback_lock.unlock();
    H3DUtil::Console( LogLevel::Error ) << "Unknown exception" << std::endl;
    throw Exception::QuitAPI();
  }
  
  callback_lock.unlock();
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepEnd("Scene_callbacks");
#endif  

  #ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::stepEnd("H3D_scene_loop");
  std::stringstream profiled_result_scene_temp;
  H3DUtil::H3DTimer::end("H3D_scene",profiled_result_scene_temp);
  if(!profiled_result_scene_temp.str().empty())
  {
    this->H3D_scene_result.setResult(ThreadBase::getCurrentThreadId(),profiled_result_scene_temp.str());
  }
  
 
  profiledResult->setValueFromString(this->generateProfileResult());

#endif

  Anchor::replaceSceneRoot( this );

  if ( X3DProgrammableShaderObject::use_bindless_textures ) {
    // Periodically check if there are any resident textures which
    // have not been used recently, and make them non-resident
    H3DTime unused_texture_time ( 5 );
    if( default_settings ) {
      GraphicsOptions* gfx_options;
      default_settings->getOptionNode( gfx_options );
      if ( gfx_options ) {
        unused_texture_time= gfx_options->bindlessTexturesUnusedTime->getValue();
      }
    }

    H3DTime cur_time= time->getValue();
    if ( cur_time - last_unused_texture_check > unused_texture_time ) {
      H3DSingleTextureNode::makeUnusedTexturesNonResident ( unused_texture_time );
      last_unused_texture_check= cur_time;
    }
  }
}



/////////////////////////////////////////////////////////////////////////////
//
// Scene member functions
//

Scene::Scene( Inst< SFChildNode >  _sceneRoot,
              Inst< MFWindow    >  _window,
              Inst< SFFloat     >  _frameRate,
              Inst< SFFloat     >  _maxFrameRate
#ifdef HAVE_PROFILER
              ,
              Inst< MFString    >  _profiledResult 
#endif
              ) :
  sceneRoot( _sceneRoot ),
  window   ( _window    ),
  frameRate( _frameRate ),
  maxFrameRate ( _maxFrameRate ),
#ifdef HAVE_PROFILER
  profiledResult( _profiledResult ),
  exclusive_times_update_time ( 0 ),
#endif
  active( true ),
  last_traverseinfo( NULL ),
  SAI_browser( NULL ),
  shadow_caster( new ShadowCaster ),
  last_unused_texture_check ( 0 ) {

  SAI_browser = this;
#ifdef HAVE_PROFILER
  H3DUtil::H3DTimer::setEnabled("H3D_scene",true);
  H3DUtil::H3DTimer::setInterval("H3D_scene",10);
#endif //HAVE_PROFILER
  if( scenes.empty() ) {
    H3DUtil::ThreadPool::global_pool.resize( 8 ); 
  }
  scenes.insert( this );
  
  
  shadow_caster->algorithm->setValue( "ZFAIL" );

  type_name = "Scene";
  database.initFields( this );
  Scene::eventSink->setName( "Scene::eventSink" );
  Scene::time->setName( "Scene::time" );
  time->setAccessType( Field::OUTPUT_ONLY );
  frameRate->setValue( 0, id );
  maxFrameRate->setValue ( -1 );
  ThreadBase::setThreadName( ThreadBase::getMainThreadId(), 
                             "H3D API Main Thread" );
  Scene::time->route( eventSink );
}

Scene::~Scene() {
  scenes.erase( this );
  if( scenes.empty() ) {
    ThreadPool::global_pool.resize(0);
  }
  setSceneRoot( NULL );

  if( last_traverseinfo )
    delete last_traverseinfo;
#ifdef HAVE_XERCES
  ProfilesAndComponents::destroy();
#endif
}

#ifdef HAVE_GLUT
void Scene::mainLoop() {
  GLUTWindow::initGLUT();
  glutIdleFunc( SceneInternal::idle );
  glutMainLoop();
}
#endif

void Scene::EventSink::update() {
  for( unsigned int i = 0; i < routes_in.size(); ++i ) {
    if( PeriodicUpdateField *pf = 
        dynamic_cast< PeriodicUpdateField * >( routes_in[i] ) ) {
      if( pf->timeToUpdate() ) routes_in[i]->upToDate();
    } else {
      routes_in[i]->upToDate();
    }
  }
}

void Scene::loadSceneRoot( const string &url ) {
  SAI::SAIScene *scene = new SAI::SAIScene;
  try {
  // TODO: fill out scene with all values
  scene->root_node.reset( X3D::createX3DFromURL( url,
                                                 &scene->named_nodes,
                                                 &scene->exported_nodes,
                                                 &scene->protos ) );
  } catch(...) {
    delete scene;
    throw;
  }
  SAI_browser.replaceWorld( scene );
}

void Scene::setSceneRoot( SAI::SAIScene *scene ) {
  SAI::ExecutionContext::cleanUp();
  SAI_browser.replaceWorld( scene );
}


void Scene::addProgramSetting( Field *field,
                               const string &name,
                               const string &section ) {
  string setting_name = name == "" ? field->getName() : name;
  program_settings.push_back( ProgramSetting( field, setting_name, section ) );
  field->markProgramSetting( true );
  for( ProgramSettingsCallbackList::iterator i = program_settings_callbacks.begin();
       i != program_settings_callbacks.end(); ++i ) {
    (*i).first( ADD_SETTING, program_settings.back(), (*i).second );
  }
}


bool Scene::removeProgramSetting( const string &name,
                                  const string &section ) {
  Field *f = getProgramSetting( name, section );
  return Scene::removeProgramSetting( f );
}


Field *Scene::getProgramSetting( const string &name,
                                 const string &section ) {
  Field *f = NULL;
  for( Scene::SettingsIterator i = program_settings.begin();
         i != program_settings.end(); ++i ) {
    const Scene::ProgramSetting &setting = *i;
    if( setting.name == name &&
        setting.section== section ) {
      f = setting.field;
      break;
    }
  }
  return f;
}


Scene::SettingsIterator Scene::programSettingsBegin() {
  return program_settings.begin();
}

/// Get an iterator to the end of the program settings. 
Scene::SettingsIterator Scene::programSettingsEnd() {
  return program_settings.end();
}

Scene::ProgramSettingsCallbackId Scene::addProgramSettingsCallback( ProgramSettingsCallbackFunc func, 
                                                             void *data ) {
  program_settings_callbacks.push_back( make_pair( func, data ) );
  return program_settings_callbacks.back();
}

bool Scene::removeProgramSettingsCallback( Scene::ProgramSettingsCallbackId id ) {
  ProgramSettingsCallbackList::iterator i = 
    std::find( 
               program_settings_callbacks.begin(), 
               program_settings_callbacks.end(), id );
  if( i != program_settings_callbacks.end() ) {
    program_settings_callbacks.erase( i );
    return true;
  } else {
    return false;
  }
}

void Scene::clearProgramSettingsCallbacks() {
  program_settings_callbacks.clear();
}

/// Remove all program settings referring to a field. 
///

bool Scene::removeProgramSetting( Field *field ) {
  bool removed = false;
  for( Scene::SettingsIterator i = program_settings.begin();
         i != program_settings.end(); ++i ) {
    Scene::ProgramSetting setting = *i;
    if( setting.field == field ) {
      program_settings.erase( i );
      field->markProgramSetting( false );
      removed = true;
      // call callbacks
      for( ProgramSettingsCallbackList::iterator j = program_settings_callbacks.begin();
           j != program_settings_callbacks.end(); ++j ) {
        (*j).first( REMOVE_SETTING, setting, (*j).second );
      }
      
      // remove any more existants of field
      Scene::removeProgramSetting( field );
      break;
    }
  }
  return removed;
}

H3D::Node* Scene::findChildNode(H3D::X3DGroupingNode *group, const std::string &nodeType, const std::string &nodeName){
  if(group)
  {
    const NodeVector &c = group->children->getValue();
    for( unsigned int i = 0; i < c.size(); ++i ) {
      Node* n = c[i];
      if( n ){
        if (n->getTypeName() == nodeType)
        {
          if ( nodeName == "" )
            return n;
          else if (nodeName == n->getName())
            return n;
        }
      }
    }
    for( unsigned int i = 0; i < c.size(); ++i ) {
      if( c[i]){
        H3D::X3DGroupingNode *childgroup = dynamic_cast<H3D::X3DGroupingNode*>(c[i]);
        if (childgroup){
          H3D::Node *node = findChildNode(childgroup, nodeType, nodeName);
          if (node)
            return node;
        }
      }
    }
    return NULL;
  }
  else
    return NULL;
}

H3D::Node* Scene::findNodeType(H3D::Node *node, const std::string &nodeType, const std::string &nodeName){
  if (node)
  {
    H3D::X3DGroupingNode* group = dynamic_cast<H3D::X3DGroupingNode*>(node);
    if (group)
    {
      const NodeVector &c = group->children->getValue();
      for( unsigned int i = 0; i < c.size(); ++i ) {
        Node* n = c[i];
        if( n ){
          if (n->getTypeName() == nodeType)
          {
            if ( nodeName == "" )
              return n;
            else if (nodeName == n->getName())
              return n;
          }
        }
      }
      for( unsigned int i = 0; i < c.size(); ++i ) {
        if( c[i] ){
          H3D::Node *_node = findNodeType(c[i], nodeType, nodeName);
          if( _node )
            return _node;
        }
      }
      return NULL;
    }
    H3D::Shape* shape = dynamic_cast<H3D::Shape*>(node);
    if (shape)
    {
      H3D::X3DGeometryNode* geom = shape->geometry->getValue();
      if (geom){
        if (geom->getTypeName() == nodeType)
        {
          if ( nodeName == "" )
            return geom;
          else if (nodeName == geom->getName())
            return geom;
        }
      }
      H3D::X3DAppearanceNode *appearance = shape->appearance->getValue();
      if (appearance){
        if (appearance->getTypeName() == nodeType)
        {
          if ( nodeName == "" )
            return appearance;
          else if (nodeName == appearance->getName())
            return appearance;
        }
        else
        {
          H3D::Node *_node = findNodeType(appearance, nodeType, nodeName);
          if( _node )
            return _node;
        }
      }
      return NULL;
    }
    H3D::Appearance* appearance = dynamic_cast<H3D::Appearance*>(node);
    if (appearance)
    {
      H3D::X3DMaterialNode *material = appearance->material->getValue();
      if (material){
        if (material->getTypeName() == nodeType)
        {
          if ( nodeName == "" )
            return material;
          else if (nodeName == material->getName())
            return material;
        }
        else
          return NULL;
      }
      return NULL;
    }
    H3D::Inline* inlinenode = dynamic_cast<H3D::Inline*>(node);
    if (inlinenode)
    {
      if( inlinenode->load->getValue() ) {
        for( unsigned int i = 0; i < inlinenode->loadedScene->size(); ++i ) {
          Group *g = inlinenode->loadedScene->getValueByIndex( i );
          if( g ){
            if (g->getTypeName() == nodeType)
            {
              if ( nodeName == "" )
                return g;
              else if (nodeName == g->getName())
                return g;
            }
          }
        }
        for( unsigned int i = 0; i < inlinenode->loadedScene->size(); ++i ) {
          Group *g = inlinenode->loadedScene->getValueByIndex( i );
          if( g){
            H3D::Node *_node = findNodeType(g, nodeType, nodeName);
            if( _node )
              return _node;
          }
        }
      }
      return NULL;
    }


    return NULL;
  }
  else
    return NULL;
}
#ifdef HAVE_PROFILER 
std::string Scene::profiledResultData::getResult()
{
  /*if(this->profiled_result_string.empty())
    return "";*/
  return this->thread_debug_string+'\n'+this->profiled_result_string;
}
#endif
