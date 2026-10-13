classdef RelaxedCallback < casadi.Callback
 properties
  fun; nx; ny; jacobianMode; child;
 end
 methods
  function self=RelaxedCallback(name,fun,nx,ny,jacobianMode)
   self@casadi.Callback(); self.fun=fun;self.nx=nx;self.ny=ny;self.jacobianMode=jacobianMode;
   self.construct(name,struct('enable_fd',false));
  end
  function n=get_n_in(self),n=1+double(self.jacobianMode);end
  function n=get_n_out(~),n=1;end
  function sp=get_sparsity_in(self,i)
   if i==0,sp=casadi.Sparsity.dense(self.nx,1);else,sp=casadi.Sparsity.dense(self.ny,1);end
  end
  function sp=get_sparsity_out(self,~)
   if self.jacobianMode,sp=casadi.Sparsity.dense(self.ny,self.nx);else,sp=casadi.Sparsity.dense(self.ny,1);end
  end
  function out=eval(self,arg)
   [v,J]=self.fun(full(arg{1}));if self.jacobianMode,out={casadi.DM(J)};else,out={casadi.DM(v)};end
  end
  function yes=has_jacobian(self),yes=~self.jacobianMode;end
  function f=get_jacobian(self,name,~,~,~)
   self.child=tsfs.RelaxedCallback(name,self.fun,self.nx,self.ny,true);f=self.child;
  end
 end
end
