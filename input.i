[Mesh]
  [gen]
    type = GeneratedMeshGenerator
    dim = 2
    xmin = 0
    xmax = 0.01
    ymin = 0
    ymax = 0.01
    nx = 50
    ny = 50
  []
[]

[Variables]
  [./c]
    order = FIRST
    family = LAGRANGE
   # scaling = 1e+04
    [./InitialCondition]
      type = ConstantIC
      value = 0.5
    [../]
  [../]
  [./w0]
    order = FIRST
    family = LAGRANGE
  [../]
  [./disp_x]
    order = FIRST
    family = LAGRANGE
  [../]
  [./disp_y]
    order = FIRST
    family = LAGRANGE
  [../]
#F
  [./F11]
    order = FIRST
    family = LAGRANGE
  [../]
  [./F12]
    order = FIRST
    family = LAGRANGE
  [../]
  [./F21]
    order = FIRST
    family = LAGRANGE
  [../]
  [./F22]
    order = FIRST
    family = LAGRANGE
  [../]
#Define LM
  [./rho11]
    order = FIRST
    family = LAGRANGE
  [../]
  [./rho12]
    order = FIRST
    family = LAGRANGE
  [../]
  [./rho21]
    order = FIRST
    family = LAGRANGE
  [../]
  [./rho22]
    order = FIRST
    family = LAGRANGE
  [../]
[]

[Kernels]
  [./c_res]
    type = SplitCHParsed
    variable = c
    f_name = F
    kappa_name = kappa_c
    w = w0
    args = 'F11 F22 F12 F21'
  [../]
  [./w_res]
    type = SplitCHWRes
    variable = w0
    mob_name = M
    args = c
  [../]
  [./time]
    type = CoupledTimeDerivative
    variable = w0
    v = c
 [../]
  [./grad_PK1_x]
    type = PK1Divergence_2D_V_C
    component = 0
    variable  = disp_x
    #Coupled variables
    u1 = disp_x
    u2 = disp_y
    rho_11 = rho11
    rho_22 = rho22
    rho_12 = rho12
    rho_21 = rho21
    s = c
  [../]
  [./grad_PK1_y]
    type = PK1Divergence_2D_V_C
    component = 1
    variable  = disp_y
    #Coupled variables
    u1 = disp_x
    u2 = disp_y
    rho_11 = rho11
    rho_22 = rho22
    rho_12 = rho12
    rho_21 = rho21
    s = c
  [../]
  [./grad_ThirdorderStress_xx]
    type = Thirdorderstress3DivergenceF_2D
    component_i = 0
    component_j = 0
    variable  = F11
    #Coupled variables
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
    rho_11 = rho11
    rho_22 = rho22
    rho_12 = rho12
    rho_21 = rho21
  [../]
  [./grad_ThirdorderStress_xy]
    type = Thirdorderstress3DivergenceF_2D
    component_i = 0
    component_j = 1
    variable  = F12
    #Coupled variables
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
    rho_11 = rho11
    rho_22 = rho22
    rho_12 = rho12
    rho_21 = rho21
  [../]
  [./grad_ThirdorderStress_yx]
    type = Thirdorderstress3DivergenceF_2D
    component_i = 1
    component_j = 0
    variable  = F21
    #Coupled variables
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
    rho_11 = rho11
    rho_22 = rho22
    rho_12 = rho12
    rho_21 = rho21
  [../]
  [./grad_ThirdorderStress_yy]
    type = Thirdorderstress3DivergenceF_2D
    component_i = 1
    component_j = 1
    variable  = F22
    #Coupled variables
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
    rho_11 = rho11
    rho_22 = rho22
    rho_12 = rho12
    rho_21 = rho21
  [../]
  [./LM_xx]
    type = LMF_2D
    component_i = 0
    component_j = 0
    variable  = rho11
    #Coupled variables
    u1 = disp_x
    u2 = disp_y
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
  [../]
  [./LM_xy]
    type = LMF_2D
    component_i = 0
    component_j = 1
    variable  = rho12
    #Coupled variables
    u1 = disp_x
    u2 = disp_y
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
  [../]
  [./LM_yx]
    type = LMF_2D
    component_i = 1
    component_j = 0
    variable  = rho21
    #Coupled variables
    u1 = disp_x
    u2 = disp_y
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
  [../]
  [./LM_yy]
    type = LMF_2D
    component_i = 1
    component_j = 1
    variable  = rho22
    #Coupled variables
    u1 = disp_x
    u2 = disp_y
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
  [../]
[]

[AuxVariables]
  [./hystress]

    family = MONOMIAL

    order = FIRST

  [../]
  [./maxP]

    family = MONOMIAL

    order = FIRST
  [../]
  [./minP]

    family = MONOMIAL

    order = FIRST
  [../]
  [./stress_xx]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./stress_yy]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./stress_xy]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./stress_yx]
    order = CONSTANT
    family = MONOMIAL
  [../]
  [./f_density]   
    order = CONSTANT
    family = MONOMIAL
  [../]
[]
[AuxKernels]
 [./gethystress]

    type = RankTwoScalarAux

    rank_two_tensor = Ch_2D_V

    variable = hystress

    scalar_type = Hydrostatic
  [../]
 [./getmaxP]

    type = RankTwoScalarAux


    rank_two_tensor = Ch_2D_V

    variable = maxP

    scalar_type = MaxPrincipal

  [../]
 [./getminP]

    type = RankTwoScalarAux


    rank_two_tensor = Ch_2D_V

    variable = minP

    scalar_type = MinPrincipal

  [../]
  [./stress_xx]
    type = RankTwoAux
    variable = stress_xx
    rank_two_tensor = Ch_2D_V
    index_j = 0
    index_i = 0
    execute_on = timestep_end
  [../]
  [./stress_yy]
    type = RankTwoAux
    variable = stress_yy
    rank_two_tensor = Ch_2D_V
    index_j = 1
    index_i = 1
    execute_on = timestep_end
  [../]
  [./stress_xy]
    type = RankTwoAux
    variable = stress_xy
    rank_two_tensor = Ch_2D_V
    index_j = 0
    index_i = 1
    execute_on = timestep_end
  [../]
  [./stress_yx]
    type = RankTwoAux
    variable = stress_yx
    rank_two_tensor = Ch_2D_V
    index_j = 1
    index_i = 0
    execute_on = timestep_end
  [../]
  # Calculates the energy density by combining the local and gradient energies
  [./f_density]   
    type = TotalFreeEnergy
    variable = f_density
    f_name = 'F'
    kappa_names = 'kappa_c'
    interfacial_vars = c
  [../]
[]

[Materials]
  [./kappa_c]
    type = GenericConstantMaterial
    prop_names  = 'kappa_c'
    prop_values = '2.8e-5'
  [../]
  [./PK1]
    type = PK1_2D_V_C
    A_name = 1001.44
    A6_name = 796.66
    K_name = 54.151
    B_name = 25860.9 
    H_name = -2736.15 
    Kappa_name = 2.8e-5
    #Coupled variables
    u1 = disp_x
    u2 = disp_y
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
    s = c
  [../]
  [./Thirdorderstress3]
    type = Thirdorderstress3F_2D
    Kappa_name = 2.8e-5
    #Coupled variables
    F_11 = F11
    F_22 = F22
    F_12 = F12
    F_21 = F21
  [../]
  [./mob]
    type = DerivativeParsedMaterial
    f_name = M
    args = 'c'
    constant_names       = 'd'
    constant_expressions = '1'
     function = 'd*c*(1-c)'
   # function = 'd'
    derivative_order = 1
    enable_jit = true
  [../]

  [./chemical_free_energy]
    type = DerivativeParsedMaterial
    f_name = F
    args = 'c F11 F22 F12 F21'
    constant_names       = 'alpha  beta1  beta2  beta3  H  A  A6  B  K' #H=A,A=A_bulk,A6=A_shear,B=C+A_bulk*V*V,K=A_bulk*V
    constant_expressions = '-579.454  -926.715  -927.453  -470.114  -2736.15  1001.44  796.66  25860.9  54.1513'
    function = alpha*c+c*log(c)+(1.0-c)*log(1.0-c)+c*(1.0-c)*(beta1+beta2*(1.0-2.0*c)+beta3*(1.0-2.0*c)^2)+(-0.748088+c)*H*1.0/8.0*(F11^2+F21^2-F12^2-F22^2)^2+A*1.0/8.0*(F11^2+F21^2+F12^2+F22^2-2.0)^2+A6*1.0/2.0*(F11*F12+F21*F22)^2+1.0/64.0*B*(F11^2+F21^2-F12^2-F22^2)^4-2*K*0.353553*(F11^2+F21^2+F12^2+F22^2-2.0)*1.0/8.0*(F11^2+F21^2-F12^2-F22^2)^2
    derivative_order = 2
    enable_jit = true
  [../]
[]

[BCs]
[./galvanostat_bottom]
    type = galvanostatPpNBC
    variable = w0
    boundary = bottom
    Da = '5.6574e-3'
    I = 3.06875
    p_var = w0
    c_var = c
    intp_postprocessor_bottom = int_P_bottom 
    intn_postprocessor_bottom = int_N_bottom
    intp_postprocessor_left = int_P_left
    intn_postprocessor_left = int_N_left
    intp_postprocessor_right = int_P_right
    intn_postprocessor_right = int_N_right
    intp_postprocessor_top = int_P_top
    intn_postprocessor_top = int_N_top
  [../]
  [./galvanostat_left]
    type = galvanostatPpNBC
    variable = w0
    boundary = left
    Da = '5.6574e-3'
    I = 3.06875
    p_var = w0
    c_var = c
    intp_postprocessor_bottom = int_P_bottom
    intn_postprocessor_bottom = int_N_bottom
    intp_postprocessor_left = int_P_left
    intn_postprocessor_left = int_N_left
    intp_postprocessor_right = int_P_right
    intn_postprocessor_right = int_N_right
    intp_postprocessor_top = int_P_top
    intn_postprocessor_top = int_N_top
  [../]
  [./galvanostat_right]
    type = galvanostatPpNBC
    variable = w0
    boundary = right
    Da = '5.6574e-3'
    I = 3.06875
    p_var = w0
    c_var = c
    intp_postprocessor_bottom = int_P_bottom
    intn_postprocessor_bottom = int_N_bottom
    intp_postprocessor_left = int_P_left
    intn_postprocessor_left = int_N_left
    intp_postprocessor_right = int_P_right
    intn_postprocessor_right = int_N_right
    intp_postprocessor_top = int_P_top
    intn_postprocessor_top = int_N_top
  [../]
  [./galvanostat_top]
    type = galvanostatPpNBC
    variable = w0
    boundary = top
    Da = '5.6574e-3'
    I = 3.06875
    p_var = w0
    c_var = c
    intp_postprocessor_bottom = int_P_bottom
    intn_postprocessor_bottom = int_N_bottom
    intp_postprocessor_left = int_P_left
    intn_postprocessor_left = int_N_left
    intp_postprocessor_right = int_P_right
    intn_postprocessor_right = int_N_right
    intp_postprocessor_top = int_P_top
    intn_postprocessor_top = int_N_top
  [../] 
 [./right_x]
    type = DirichletBC
    preset = true
    variable = disp_x
    boundary = 'right'
    value = 0
  [../]
  [./right_y]
    type = DirichletBC
    preset = true
    variable = disp_y
    boundary = 'right'
    value = 0
  [../]
  [./left_x]
    type = DirichletBC
    preset = true
    variable = disp_x
    boundary = 'left'
    value = 0
  [../]
  [./left_y]
    type = DirichletBC
    preset = true
    variable = disp_y
    boundary = 'left'
    value = 0
  [../]
  [./top_x]
    type = DirichletBC
    preset = true
    variable = disp_x
    boundary = 'top'
    value = 0
  [../]
  [./top_y]
    type = DirichletBC
    preset = true
    variable = disp_y
    boundary = 'top'
    value = 0
  [../]
  [./bottom_x]
    type = DirichletBC
    preset = true
    variable = disp_x
    boundary = 'bottom'
    value = 0
  [../]
  [./bottom_y]
    type = DirichletBC
    preset = true
    variable = disp_y
    boundary = 'bottom'
    value = 0
  [../]
[]


[Preconditioning]
  # active = ' '
  [./SMP]
    type = SMP
    full = true
  [../]
[]

  
[Postprocessors]
  [./total_solute]
    type = ElementAverageValue
    variable = c
  [../]
  [./step_size]             # Size of the time step
    type = TimestepSize
  [../]
  [./nodes]                 # Number of nodes in mesh
    type = NumNodes
  [../]
  [./cc_min]
    type = NodalExtremeValue
    variable = c
    value_type = min
  [../]
  [./cc_max]
    type = NodalExtremeValue
    variable = c
    value_type = max
  [../]
  [./F11_min]
    type = NodalExtremeValue
    variable = F11
    value_type = min
  [../]
  [./F11_max]
    type = NodalExtremeValue
    variable = F11
    value_type = max
  [../]
  [./F22_min]
    type = NodalExtremeValue
    variable = F22
    value_type = min
  [../]
  [./F22_max]
    type = NodalExtremeValue
    variable = F22
    value_type = max
  [../]
  [./F12_min]
    type = NodalExtremeValue
    variable = F12
    value_type = min
  [../]
  [./F12_max]
    type = NodalExtremeValue
    variable = F12
    value_type = max
  [../]
  [./F21_min]
    type = NodalExtremeValue
    variable = F21
    value_type = min
  [../]
  [./F21_max]
    type = NodalExtremeValue
    variable = F21
    value_type = max
  [../]
  [./total_energy]          # Total free energy at each timestep
    type = ElementIntegralVariablePostprocessor
    variable = f_density
  [../]
[./int_P_bottom]
    type = SIntGalvanostatP
    boundary = bottom
    num_area = 1
    variable = w0
    p_var = w0
    c_var = c
    execute_on = 'initial timestep_begin timestep_end'
  [../]
  [./int_N_bottom]
    type = SIntGalvanostatN
    boundary = bottom
    num_area = 1
    variable = c
    c_var = c
    execute_on = 'initial timestep_begin timestep_end'
  [../]
  [./int_P_right]
    type = SIntGalvanostatP
    boundary = right
    num_area = 1
    variable = w0
    p_var = w0
    c_var = c
    execute_on = 'initial timestep_begin timestep_end'
  [../]
  [./int_N_right]
    type = SIntGalvanostatN
    boundary = right
    num_area = 1
    variable = c
    c_var = c
    execute_on = 'initial timestep_begin timestep_end'
  [../]
  [./int_P_left]
    type = SIntGalvanostatP
    boundary = left
    num_area = 1
    variable = w0
    p_var = w0
    c_var = c
    execute_on = 'initial timestep_begin timestep_end'
  [../]
  [./int_N_left]
    type = SIntGalvanostatN
    boundary = left
    num_area = 1
    variable = c
    c_var = c
    execute_on = 'initial timestep_begin timestep_end'
  [../]
  [./int_P_top]
    type = SIntGalvanostatP
    boundary = top
    num_area = 1
    variable = w0
    p_var = w0
    c_var = c
    execute_on = 'initial timestep_begin timestep_end'
  [../]
  [./int_N_top]
    type = SIntGalvanostatN
    boundary = top
    num_area = 1
    variable = c
    c_var = c
    execute_on = 'initial timestep_begin timestep_end'
  [../]
  [./maxP_min]
    type = ElementExtremeValue
    variable = maxP
    value_type = min
  [../]
  [./maxP_max]
    type = ElementExtremeValue
    variable = maxP
    value_type = max
  [../]
  [./minP_min]
    type = ElementExtremeValue
    variable = minP
    value_type = min
  [../]
  [./minP_max]
    type = ElementExtremeValue
    variable = minP
    value_type = max
  [../]
  [./hystress_min]
    type = ElementExtremeValue
    variable = hystress
    value_type = min
  [../]
  [./hystress_max]
    type = ElementExtremeValue
    variable = hystress
    value_type = max
  [../]
[]


[Executioner]
  type = Transient
   [./TimeIntegrator]
      type = ImplicitEuler
    # type = BDF2
    # type = CrankNicolson
    # type = ImplicitMidpoint
    # type = LStableDirk2
    # type = LStableDirk3
    # type = LStableDirk4
    # type = AStableDirk4
    #
    # Explicit methods
    # type = ExplicitEuler
    # type = ExplicitMidpoint
    # type = Heun
    # type = Ralston
  [../]

      solve_type = 'PJFNK'
      petsc_options_iname = '-pc_type  -sub_pc_type '
      petsc_options_value = 'none      lu'
   #  petsc_options= '-ksp_monitor_singular_value' 

   # solve_type = 'PJFNK'
   # petsc_options_iname = '-ts_adapt_type  -snes_type  -ts_max_snes_failures -snes_max_it  -snes_max_funcs -ksp_type  -pc_type  -ksp_ksp_type  -ksp_pc_type -ksp_gmres_restart -ksp_max_it'
   # petsc_options_value = 'none newtontr 500 100 50000 fgmres none gmres jacobi 1200 2401'
   # petsc_options= '-ts_monitor -snes_monitor -snes_converged_reason -ksp_converged_reason -snes_mf -ksp_gmres_preallocate -ksp_gmres_modifiedgramschmidt'


   # solve_type = 'PJFNK'
   # petsc_options = -snes_ksp_ew
   # petsc_options_iname = '-pc_type -sub_pc_type -pc_asm_overlap -ksp_gmres_restart'
   # petsc_options_value = 'asm lu 1 101'

 # petsc_options_iname = '-pc_type -ksp_grmres_restart -sub_ksp_type -sub_pc_type -pc_asm_overlap'
 # petsc_options_value = 'asm      31                  preonly      ilu           2'

  #Terms for debugging
#  petsc_options = '-ksp_monitor_true_residual -ksp_compute_singularvalues' 
#  petsc_options = '-snes_converged_reason -ksp_converged_reason'
#  l_max_its  = 10
#  petsc_options_iname = '-pc_type -pc_hypre_type -ksp_gmres_restart'
#  petsc_options_value = 'hypre    boomeramg      100'
#  petsc_options_iname = '-ksp_gmres_restart'
#  petsc_options_value = '100'
#  petsc_options = '-snes_ksp_ew -ksp_monitor_true_residual -ksp_compute_singularvalues'# -pc_svd_monitor'
#  petsc_options = '-ksp_monitor_true_residual -ksp_compute_singularvalues'# -pc_svd_monitor'
#  petsc_options_iname = '-pc_type -sub_pc_type -pc_asm_overlap -ksp_gmres_restart -print_linear_residuals'# -ksp_view_mat'
#  petsc_options_value = 'asm      lu           1               101                false                  '# binary'

  l_max_its = 30
 # nl_max_its = 100
  l_tol = 1.0e-4
  nl_rel_tol = 1.0e-4
  nl_abs_tol = 1.0e-4
  start_time = 0.01
  end_time = 2
  #num_steps = 10
  dt = 1e-6
  dtmin = 1e-50
   automatic_scaling = true
   # compute_scaling_once=false

  [./TimeStepper]
    type = IterationAdaptiveDT
    dt = 1e-6 # Initial time step.  In this simulation it changes.
    optimal_iterations = 72 #Time step will adapt to maintain this number of nonlinear iterations
  [../]
[]

[Debug]
  show_var_residual_norms = true
[]

[Outputs]
  exodus = true
  csv = true
  interval = 200
  #vtk = true
  checkpoint = true
[]