"""Shared Motion binding; no ROS imports until the supplied module is used."""
from common import now
from controller import publication_ownership_ok
from trajectory_guard import HorizontalPlanAuditMixin

def bind_motion(mod):
    class HorizontalMotion(HorizontalPlanAuditMixin,mod.Motion):
        audit_message_to_dict=staticmethod(mod.asdict)
        ownership_context=None
        def check_publisher_ownership(self):
            infos=self.get_publishers_info_by_topic('/crane_plus_arm_controller/joint_trajectory')
            rows=[{'node_name':p.node_name,'node_namespace':p.node_namespace} for p in infos]
            ok=publication_ownership_ok(rows,self.get_name(),self.get_namespace())
            if self.ownership_context is not None:
                manifest,persist=self.ownership_context
                manifest.setdefault('motion_publisher_checks',[]).append({'at_utc':now(),'result':'PASS' if ok else 'FAIL','expected_node_name':self.get_name(),'observed_publishers':rows})
                persist()
            if not ok:raise RuntimeError('trajectory publisher ownership not singular/verified; no send or retry')
        def verify_scene(self):
            result=super().verify_scene()
            if self.ownership_context is not None:self.check_publisher_ownership()
            return result
        def move(self,source,target,run_dir,manifest,persist,action_record=None):
            self.is_alignment=target.startswith('align_')
            self.ownership_context=(manifest,persist)
            try:
                self.check_publisher_ownership()
                return super().move(source,target,run_dir,manifest,persist,action_record=action_record)
            finally:self.is_alignment=False;self.ownership_context=None
    return HorizontalMotion
