"""Two operating modes, Mapping and Go to position, with Idle as resting state."""
from pathlib import Path
import time,json,math,numpy as np
from .features import Features
from .stereo import Stereo,wrap,optical_to_world,CAMERA_OFFSET
from .geometry import TURN_CLEAR,DRIVE_CLEARANCE
from .localization import Localizer
from .grid import Grid
class Brain:
    def __init__(self,root,backend='orb'):
        self.root=Path(root);self.features=Features(backend);self.stereo=Stereo(self.features);self.localizer=Localizer(self.features);self.grid=Grid()
        self.mode='idle';self.phase='idle';self.message='Ready. Start Mapping, or choose a target in a saved map.';self.goal=None;self.path=[];self.events=[];self.frame=None;self.current_cloud=np.empty((0,3));self.started=0;self.phase_started=0;self.scan_angle=0;self.last_yaw=None;self.blacklist=[];self.next_plan=0;self.map_folder=None;self.complete=False;self.no_frontier_scans=0;self.arrival_frames=0;self.last_progress=0;self.best_distance=math.inf;self.scan_target=2*math.pi;self.return_phase='';self.pose_origin_set=False;self.progress_pose=np.zeros(3);self.plan_not_before=0.;self.scanned_positions=[];self.motor_seen_armed=False;self.scan_direction=1.;self.weak_view_frames=0;self.good_view_frames=0;self.recovery_yaw=0.;self.recovery_attempts=0;self.last_recovery_position=None;self.recovery_target_yaw=0.;self.view_blacklist=[];self.last_good_yaw=0.;self.requested_motion=(0.,0.);self.turn_stall_since=None
    def event(self,message):
        self.message=message
        if self.events and self.events[-1]['message']==message:return
        self.events.append(dict(time=time.time(),message=message));self.events=self.events[-35:]
    def idle(self,reason='Stopped by user'):
        if self.mode=='mapping':self.save_map(False,reason)
        self.mode='idle';self.phase='idle';self.path=[];self.event(reason)
    def start_mapping(self,stamp):
        if self.mode=='mapping':self.save_map(False,'Restarted mapping')
        self.localizer=Localizer(self.features);self.grid=Grid();self.mode='mapping';self.phase='scan';self.started=stamp;self.phase_started=stamp;self.scan_angle=0;self.last_yaw=None;self.blacklist=[];self.path=[];self.goal=None;self.complete=False;self.no_frontier_scans=0;self.frame=None;self.scanned_positions=[];self.plan_not_before=stamp;self.last_progress=stamp;self.progress_pose=np.zeros(3);self.motor_seen_armed=False;self.scan_direction=1.;self.weak_view_frames=0;self.good_view_frames=0;self.recovery_attempts=0;self.last_recovery_position=None;self.recovery_target_yaw=0.;self.view_blacklist=[];self.last_good_yaw=0.;self.requested_motion=(0.,0.);self.turn_stall_since=None;self.map_folder=self.root/'Saved/Maps'/time.strftime('map_%Y%m%dT%H%M%S');self.event('Mapping: scanning from the current position. No predefined route.')
    def begin_go_to(self,goal,stamp):
        if self.mode=='mapping':self.save_map(False,'Switched to navigation')
        pointer=self.root/'Saved/Maps/latest-map.json'
        if not pointer.exists():raise ValueError('Create a map first')
        folder=self.root/'Saved/Maps'/json.loads(pointer.read_text())['directory'];grid=Grid();grid.load(folder);loc=Localizer(self.features);loc.load(folder)
        c=grid.cell(goal)
        if not grid.valid(c) or not grid.masks()[3][c]:raise ValueError('Choose a target inside mapped free space with enough clearance')
        if len(loc.keyframes)<3:raise ValueError('Map has too few stereo landmarks for reliable relocalization')
        self.grid=grid;self.localizer=loc;self.goal=np.array(goal,float);self.map_folder=folder;self.mode='go_to';self.phase='relocalizing';self.phase_started=stamp;self.started=stamp;self.path=[];self.arrival_frames=0;self.frame=None;self.plan_not_before=stamp;self.last_progress=stamp;self.progress_pose=np.zeros(3);self.motor_seen_armed=False;self.event('Locating the robot in saved stereo landmarks before planning.')
    def predict(self,state):
        if self.mode!='idle' and (state.get('fault') in ['bumper','cliff_or_invalid','tilt'] or state['sensors'].get('bumper')):
            self.localizer.invalidate();self.idle('Contact or unsafe motion detected. Position is uncertain; map updates stopped.');return
        stamp=state['sim_time_s'];sensors=state['sensors']
        blocked=self.mode!='idle' and abs(self.requested_motion[1])>.10 and abs(sensors.get('gyro_rad_s',[0,0,1])[2])<.03
        if blocked:
            if self.turn_stall_since is None:self.turn_stall_since=stamp
            elif stamp-self.turn_stall_since>3:
                self.handle_turn_stall(stamp);return
        else:self.turn_stall_since=None
        self.localizer.predict(stamp,sensors)
        yaw=self.localizer.pose[2]
        if self.last_yaw is not None and self.phase=='scan':self.scan_angle+=abs(wrap(yaw-self.last_yaw))
        self.last_yaw=yaw
    def handle_turn_stall(self,stamp):
        self.localizer.invalidate();self.idle('Turning is blocked or wheels are slipping. Position marked uncertain; partial map saved.')
    def observe(self,left,right,meta):
        frame=self.stereo.process(left,right,meta);self.frame=frame
        view_recovery=self.phase=='recover_view'
        locating=self.phase in ['relocalizing','recover_localization'] or (view_recovery and bool(self.localizer.keyframes))
        good,pose=self.localizer.update(frame,mapping=self.mode=='mapping' and not locating,relocalizing=locating)
        self.current_cloud=self.stereo.cloud(frame)
        depth_features=int((frame.points[:,2]>0).sum())
        useful=depth_features>=60 and ((good and self.localizer.visual_inliers>=50) or self.localizer.tracking=='map origin')
        self.weak_view_frames=0 if useful else self.weak_view_frames+1
        self.good_view_frames=self.good_view_frames+1 if useful else 0
        if useful:self.last_good_yaw=self.localizer.pose[2]
        if view_recovery and (good or (not locating and self.good_view_frames>=3)):
            self.scanned_positions.append(self.localizer.pose[:2].copy())
            self.phase='plan';self.plan_not_before=frame.stamp;self.last_progress=frame.stamp;self.progress_pose=self.localizer.pose.copy()
            self.event('Position verified against saved landmarks. Choosing a clearer viewpoint.')
        elif good and locating:
            self.phase='plan' if self.mode=='go_to' else 'scan';self.phase_started=frame.stamp;self.scan_angle=0;self.last_progress=frame.stamp;self.progress_pose=self.localizer.pose.copy();self.event('Position recognized from saved 3D landmarks.')
        if self.localizer.motion_inconsistent and self.mode!='idle' and not locating:
            self.phase='recover_localization';self.phase_started=frame.stamp;self.localizer.previous=None;self.localizer.relocalization_hits=0
            self.event('Cameras and motion sensors disagree. Stopped to re-establish position before updating the map.')
            return
        if self.mode in ['mapping','go_to'] and not locating and self.localizer.uncertainty<.12 and (good or self.localizer.tracking=='map origin'):
            self.grid.integrate(self.current_cloud,pose)
        if self.mode=='go_to' and self.phase=='confirm_arrival':
            if good and np.linalg.norm(self.goal-self.localizer.pose[:2])<.10:self.arrival_frames+=1
            else:self.arrival_frames=0
            if self.arrival_frames>=5:self.mode='idle';self.phase='idle';self.path=[];self.event('Arrived. Position checked across five camera updates; now Idle.')
    def save_map(self,complete,reason):
        if self.map_folder is None:return
        self.localizer.save(self.map_folder);self.grid.save(self.map_folder)
        manifest=dict(complete=complete,reason=reason,unresolved_frontier_count=len(self.blacklist),scope='observed reachable space; no claim about unseen closed rooms',pose_confidence_is_heuristic=True,feature_backend=self.features.backend,keyframes=len(self.localizer.keyframes),**self.grid.summary())
        (self.map_folder/'manifest.json').write_text(json.dumps(manifest,indent=2));(self.root/'Saved/Maps/latest-map.json').write_text(json.dumps({'directory':self.map_folder.name}))
    def plan(self,stamp):
        pose=self.localizer.pose
        if self.mode=='mapping':
            candidates=self.grid.frontiers(pose,self.blacklist+self.scanned_positions+self.view_blacklist)
            if not candidates:
                stats=self.grid.summary();self.no_frontier_scans+=1
                if self.no_frontier_scans>=3:
                    unvisited_candidates=self.grid.frontiers(pose,self.blacklist)
                    complete=not self.blacklist and not unvisited_candidates and stats['free_area_m2']>1 and stats['uncertain_observed_fraction']<.12 and self.localizer.uncertainty<.08
                    self.save_map(complete,'No reachable frontier after three scans')
                    self.mode='idle';self.phase='idle';self.complete=complete
                    self.event('Mapping finished for observed reachable space.' if complete else 'Mapping paused: no safe frontier, but uncertainty remains. Map saved as partial.');return
                self.phase='scan';self.scan_angle=0;self.phase_started=stamp;self.event('Re-scanning uncertain boundaries before deciding whether mapping is complete.');return
            def supported(candidate):
                score,xy,path=candidate;heading=math.atan2(path[-1][1]-path[max(0,len(path)-4)][1],path[-1][0]-path[max(0,len(path)-4)][0])
                return score*(.4+.6*self.localizer.view_support(xy,heading))
            _,new_goal,self.path=max(candidates,key=supported)
            changed=self.goal is None or np.linalg.norm(new_goal-self.goal)>.15
            self.goal=new_goal;self.no_frontier_scans=0;
            if changed:self.best_distance=math.inf
            self.event('Exploring a frontier chosen from the camera-built map.')
        else:
            self.path=self.grid.path(pose[:2],self.goal)
            if not self.path:self.idle('No path through known free space. The target is blocked or not connected to this position.');return
            self.event('Path planned in the saved map; following with live position checks.')
        self.phase='follow';self.phase_started=stamp;self.next_plan=stamp+1
    def command(self,state):
        stamp=state['sim_time_s'];loc=self.localizer;pose=loc.pose
        if self.mode=='idle':return 0.,0.
        if state.get('armed',True):self.motor_seen_armed=True
        elif self.motor_seen_armed or stamp-self.started>2:
            self.idle('Motor stopped ('+state.get('fault','disarmed')+'). Mapping/navigation stopped; progress saved.');return 0.,0.
        if state['fault'] in ['bumper','cliff_or_invalid','tilt']:
            self.localizer.invalidate();self.idle('Stopped: '+state['fault']+'. The map may be incomplete.');return 0.,0.
        if stamp-self.started>900:self.idle('Mapping/navigation time budget reached; unfinished work saved.');return 0.,0.
        if self.frame is None or stamp-self.frame.stamp>.8:return 0.,0.
        if self.phase=='recover_view':
            elapsed=stamp-self.phase_started
            if elapsed>55:
                self.idle('Could not recover a useful camera view safely. Partial map saved.');return 0.,0.
            # Return toward the last good view first; if it is no longer
            # recognizable, look around the room without translating.
            if elapsed>8:return 0.,self.scan_direction*.16
            offsets=[0.,-.20,.20,-.40,.40];target=self.recovery_target_yaw+offsets[min(4,int(elapsed/3))]
            error=wrap(target-pose[2]);return 0.,0. if abs(error)<.035 else float(np.clip(1.5*error,-.14,.14))
        if self.phase in ['relocalizing','recover_localization']:
            if stamp-self.phase_started>55:self.idle('Could not locate the robot confidently in the saved map.');return 0.,0.
            # First try stationary recognition; no blind continuation after slip.
            if stamp-self.phase_started<3:return 0.,0.
            if not self.rotation_clear():self.idle('Position uncertain and no safe turning clearance.');return 0.,0.
            return 0.,.14
        stale_visual=stamp-loc.last_visual_time
        if self.phase in ['scan','follow'] and self.weak_view_frames>=3:
            if self.last_recovery_position is None or np.linalg.norm(pose[:2]-self.last_recovery_position)>.30:self.recovery_attempts=0
            self.last_recovery_position=pose[:2].copy();self.recovery_attempts+=1
            if self.recovery_attempts>6:
                self.idle('Repeated loss of useful camera features. Partial map saved.');return 0.,0.
            # Retrace the last scanning direction to reacquire the view just lost.
            if self.phase=='follow' and self.goal is not None:self.view_blacklist.append(self.goal.copy())
            turn=self.requested_motion[1] if abs(self.requested_motion[1])>.03 else self.scan_direction
            self.recovery_target_yaw=self.last_good_yaw if abs(wrap(self.last_good_yaw-pose[2]))>.03 else wrap(pose[2]-.25*np.sign(turn))
            self.scan_direction=-float(np.sign(turn));self.phase='recover_view';self.phase_started=stamp;self.good_view_frames=0;loc.previous=None;loc.relocalization_hits=0;loc.uncertainty=max(.2,loc.uncertainty)
            self.event('Camera view has too few reliable features. Turning back toward the last useful view.');return 0.,0.
        if self.phase in ['follow','confirm_arrival'] and (loc.uncertainty>.18 or stale_visual>2.5):
            self.phase='recover_localization';self.phase_started=stamp;loc.relocalization_hits=0;loc.previous=None;self.event('Position confidence dropped. Stopping travel and searching for known landmarks.');return 0.,0.
        if self.phase=='scan':
            if self.scan_angle>=self.scan_target:
                self.scanned_positions.append(pose[:2].copy());self.last_progress=stamp;self.progress_pose=pose.copy();self.phase='plan';self.phase_started=stamp;self.plan_not_before=stamp;return 0.,0.
            if not self.rotation_clear():
                self.idle('Too close to an obstacle to scan safely. Partial map saved.');return 0.,0.
            if stamp-self.phase_started>65:
                self.idle('Scan made no progress. Partial map saved.');return 0.,0.
            return 0.,self.scan_direction*.16
        if self.phase=='plan':
            if stamp>=self.plan_not_before:self.plan(stamp);self.plan_not_before=stamp+.5
            return 0.,0.
        if self.phase=='confirm_arrival':
            if np.linalg.norm(self.goal-pose[:2])>=.10:
                self.phase='plan';self.arrival_frames=0;self.event('Position changed during arrival check; correcting the route.');return 0.,0.
            if stamp-self.phase_started>8:
                self.idle('Arrival could not be confirmed from camera updates.');return 0.,0.
            return 0.,0.
        if self.phase!='follow':return 0.,0.
        distance=float(np.linalg.norm(self.goal-pose[:2]))
        if distance<.10:
            if self.mode=='mapping':self.phase='scan';self.scan_angle=0;self.phase_started=stamp;self.scan_direction=1. if loc.view_support(pose[:2],pose[2]+.5)>=loc.view_support(pose[:2],pose[2]-.5) else -1.;self.save_map(False,'Exploration in progress');self.event('At a new viewpoint. Scanning the room and uncertain boundaries.')
            else:self.phase='confirm_arrival';self.phase_started=stamp;self.arrival_frames=0;self.event('At the goal. Checking position before returning to Idle.')
            return 0.,0.
        if np.linalg.norm(pose[:2]-self.progress_pose[:2])>.035 or abs(wrap(pose[2]-self.progress_pose[2]))>.12:
            self.progress_pose=pose.copy();self.last_progress=stamp
        self.best_distance=min(self.best_distance,distance)
        if stamp-self.last_progress>18:
            if self.mode=='mapping':self.blacklist.append(self.goal.copy());self.phase='scan';self.phase_started=stamp;self.scan_angle=0;self.event('No progress toward this frontier; re-scanning and choosing another.')
            else:self.idle('No progress along the path; stopped before claiming arrival.')
            return 0.,0.
        if stamp>=self.next_plan:
            updated=self.grid.path(pose[:2],self.goal);self.next_plan=stamp+1
            if not updated:self.phase='plan';return 0.,0.
            self.path=updated
        path=np.array(self.path);nearest=int(np.argmin(np.linalg.norm(path-pose[:2],axis=1)));target=path[min(len(path)-1,nearest+3)]
        error=wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2]);w=float(np.clip(1.5*error,-.35,.35));v=min(.075,distance*.45) if abs(error)<.4 else 0.
        # Check the swept forward path, rather than vetoing every turn when
        # something is anywhere in the camera's forward rectangle.
        points=self.obstacle_points()
        if abs(w)>.02 and not self.rotation_clear():
            self.idle('Too close to an obstacle to turn safely. Partial map saved.');return 0.,0.
        if v>0 and self.translation_blocked(points,v,w):
            self.grid.mark_obstacles(self.current_cloud,pose)
            v=0.
            # A safe in-place turn can align with the planned detour. Keep its
            # progress deadline; a replan must not restart the same goal forever.
            if abs(w)>.06:
                self.event('Obstacle blocks forward travel; turning along the safe detour.')
            else:
                self.phase='plan';self.plan_not_before=stamp+.5;self.event('Forward path blocked. Updating obstacles and choosing a route around them.')
        return v,w
    def obstacle_points(self):
        p=self.current_cloud
        if not len(p):return np.empty((0,2))
        # Legacy stereo backend: left camera at stereo.CAMERA_OFFSET; height band = ground clearance .. robot top.
        height=-p[:,1]+CAMERA_OFFSET[2];keep=(height>.035)&(height<.27)
        return np.column_stack((p[keep,2]+CAMERA_OFFSET[0],-p[keep,0]+CAMERA_OFFSET[1]))
    def rotation_clear(self):
        p=self.obstacle_points()
        # The round body turns about base_link: circle rule at TURN_CLEAR (a few
        # stray stereo points are tolerated).
        return len(p)==0 or int((np.linalg.norm(p,axis=1)<TURN_CLEAR).sum())<4
    @staticmethod
    def translation_blocked(points,v,w):
        if not len(points):return False
        # Include the stereo near blind zone as well as reaction/braking distance.
        stopping_distance=.12+v*.6+v*v/(2*.20)
        times=np.linspace(0,stopping_distance/max(v,.01),6)
        for dt in times:
            center=np.array([v*dt,0.]) if abs(w)<.01 else np.array([v/w*math.sin(w*dt),v/w*(1-math.cos(w*dt))])
            if int((np.linalg.norm(points-center,axis=1)<DRIVE_CLEARANCE).sum())>=4:return True
        return False
    def status(self,state=None):
        return dict(mode=self.mode,phase=self.phase,message=self.message,pose_m=self.localizer.pose.tolist(),pose_uncertainty_indicator_m=self.localizer.uncertainty,tracking=self.localizer.tracking,visual_inliers=self.localizer.visual_inliers,keyframes=len(self.localizer.keyframes),feature_backend=self.features.backend,goal_m=None if self.goal is None else self.goal.tolist(),path_m=[p.tolist() for p in self.path[::3]],map=self.grid.summary(),grid=dict(resolution=self.grid.res,size=self.grid.n,origin=self.grid.origin.tolist(),view_bounds=self.grid.view_bounds(self.localizer.pose,self.goal)),map_saved=(self.root/'Saved/Maps/latest-map.json').exists(),events=self.events,sim_time_s=state['sim_time_s'] if state else None,motor_armed=state['armed'] if state else False,motor_fault=state.get('fault','unknown') if state else 'unknown',stereo_frame=self.frame.index if self.frame else None,motion_disagreement=self.localizer.motion_inconsistent,motion_residual_m=self.localizer.motion_residual_m)
