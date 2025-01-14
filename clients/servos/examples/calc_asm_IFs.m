%
% This script calculates the ASM influence function.
%

%% General settings
%%
% clearvars;
show_act_numbering = true; %false; %
save_if = false;%true;
use_dyn_model = false;


%% GMTO model and permutation matrices
%%
% ModelFolder = fullfile(im.lfFolder,"20230131_1605_zen_30_M1_202110_ASM_202208_Mount_202111");
%ModelFolder = fullfile("/home/rconan/mnt","20240325_1556_zen_30_ASM_S1_only");
ModelFolder = fullfile("/home/rconan/mnt","20241119_1015_zen_30_ASM_S1_only");
%ModelFolder = fullfile("/home/rconan/mnt","20240322_1204_zen_30_ASM_S7_only");
% ModelFolder = fullfile(im.lfFolder,"20240401_1605_zen_30_M1_202110_ASM_202403_Mount_202305_IDOM_concreteAndFoundation_M1Fans");
FileName = "static_reduction_model.mat";

load(fullfile(ModelFolder,FileName),'inputTable','outputTable','gainMatrix');


%% Take indices and node locations from input and output tables
%%
if(contains(ModelFolder,"ASM_S1_only")), m2_seg = 1;
elseif(contains(ModelFolder,"ASM_S7_only")), m2_seg = 7;
else, m2_seg = 2; %m2_seg = 7;
end

fprintf('Performing analysis for segment #%d\n',m2_seg);

% Indexes of the voice coil force vector
vc_F_label = sprintf('MC_M2_S%d_VC_delta_F', m2_seg);
i_vc_F = inputTable(vc_F_label,:).indices{1};
% Indexes of the capacitive sensor displacements
vc_D_label = sprintf('MC_M2_S%d_VC_delta_D', m2_seg);
i_vc_D = outputTable(vc_D_label,:).indices{1};
% Indexes of the ASM shell nodes
shell_D_label = sprintf('M2_segment_%d_axial_d', m2_seg);
i_shell_D = outputTable(shell_D_label,:).indices{1};

% X-Y coordinates of the ASM magnets 
nact = inputTable(vc_F_label,1).size;
x = zeros(nact,1);
y = zeros(nact,1);

for ii = 1:nact
    label = vc_F_label;
    id = 2;
    props = inputTable(label,:).properties{1}{ii};
    %props = outputTable(label,:).properties{1}{ii};
    x(ii) = props.location(id,1);
    y(ii) = props.location(id,2);
end


% X-Y coordinates of the ASM surface nodes
nshell = outputTable(shell_D_label,1).size;
xs = zeros(nshell,1);
ys = zeros(nshell,1);

for ii = 1:nshell
    props = outputTable(shell_D_label,:).properties{1}{ii};
    xs(ii) = props.location(1,1);
    ys(ii) = props.location(1,2);
end

if(show_act_numbering)
    figure(11*m2_seg + 100); %#ok<*UNRCH>
    set(gcf,'Units','Normalized','Position',[0 0.06 0.6 0.85])
    plot(x, y, '.-');
    text(x, y, string(1:nact));
    grid on;  axis equal; axis tight;
    title(sprintf('ASM-S%d actuator numbering (%s,node#%d)',m2_seg,label,id), 'Interpreter', 'none');
end

% return

%% Calculate the stiffness matrix
%%

Psi = eye(nact) / gainMatrix(i_vc_D,i_vc_F);

%% Calculate the influence matrix
%%

if(true)
    IF = gainMatrix(i_shell_D,i_vc_F) * Psi;
else
    FileName = "modal_state_space_model_2ndOrder.mat";
    load(fullfile(ModelFolder,FileName),...
        'inputs2ModalF','modalDisp2Outputs','eigenfrequencies');
    invOM2 = diag(1./((2*pi*eigenfrequencies(4:end)).^2));
    gainMatrix_ = modalDisp2Outputs(i_vc_D,4:end) * invOM2 * inputs2ModalF(4:end,i_vc_F);
    warning('Taking static gain from dynamic model.');
    IF = gainMatrix_ * Psi;
end

tri = delaunay(xs,ys);
if(save_if)
    readme = "The file contains four other variables besides this readme. The influence matrix is saved as IF. The coordinates of the surface nodes are saved in vectors xs and ys. For tracking purposes, the variable ModelFolder has the original FE model folder used to compute the influence matrix.";
    str = split(ModelFolder,'-data/',1);    
    if_filename = sprintf("asmS%d_IF_%s",m2_seg,strtok(str(2),'zen'));
    save(if_filename,'IF','xs','ys','ModelFolder');
    fprintf('Influence matrix of segment #%d saved as %s.mat\n',m2_seg,if_filename);
end

%% Assess ASM IF
%%
u = zeros(nact,1);
this_if = 5;%675;%609;%
% this_if = [3,12,27,48,75,108,147,192,243,300,363,432,507,588,675]';%5;%675;%609;%
% this_if = [117,220,225,244,313,321,463,472,474,526,547,578,603,605,631]';
u(this_if) = 1e-6;
s = IF*u;

figure(20*m2_seg+ 1+this_if(1));
trisurf(tri,xs,ys,s,'Facecolor','interp','Linestyle','none');
hold on;
plot(x,y, 'k.'); hold off;
axis equal; axis tight; colormap('jet'); colorbar; view(2);
if(length(this_if) ~= 1), title('Poking series of actuators');
else, title('Poking actuator #'+string(this_if(1)));
end

%% Check for motion of the reference-body
%%

i_m2RB_D = outputTable("MC_M2_RB_6D",:).indices{1};
K21 = gainMatrix(i_m2RB_D,i_vc_F);
fprintf('Induced RB rbm:\n')
fprintf('%g\n',K21*u);

i_s_rbm_D = outputTable("MC_M2_lcl_6D",:).indices{1};
K31 = gainMatrix(i_s_rbm_D,i_vc_F);
fprintf('Induced aASM shell rbm:\n')
fprintf('%g\n',K31*u);

%% Adoptica format
if (adoptica) 
    in = [xs,ys];
    in_act = [x,y];
    K = Psi;
    data = gainMatrix(i_shell_D,i_vc_F) * K;
    str = split(ModelFolder,'/',1);    
    if_filename = sprintf("%s_IFs",strtok(str(end)));
    save(if_filename,'data','K','in','in_act');
end

%%
figure(20*m2_seg+ 1+this_if(1));
trisurf(tri,xs,ys,data(:,364),'Facecolor','interp','Linestyle','none');
hold on;
plot(x,y, 'k.'); hold off;
axis equal; axis tight; colormap('jet'); colorbar; view(2);