#def tune_short(self, tune= "t200 o2 a8 a4") -> None:
#        sysid = self.master.target_system
#        compid = self.master.target_component
#
#        self.master.mav.play_tune_send(sysid, compid, tune.encode())
#    
#    def tune_long(self, tune= "t100 o2 a8 a4 a8 a4") -> None:


#        sysid = self.master.target_system
#        compid = self.master.target_component
#
#        # Podział melodii na dwie części, jeśli jest dłuższa niż 30 znaków
#        tune1 = tune[:30]
#        tune2 = tune[30:]
#
#        self.master.mav.play_tune_send(sysid, compid, tune1.encode(), tune2.encode())
#
#    def play_Barka(self) -> None:
#        tune = "T140 o3 e2 p8 l4 e d e f e d c c2 p4 d2 e2 f2 f2 p16 f16 f f e d2 d2"
#        sysid = self.master.target_system
#        compid = self.master.target_component
#
#        tune1 = tune[:30]
#        tune2 = tune[30:]
#
#        self.master.mav.play_tune_send(sysid, compid, tune1.encode(), tune2.encode())