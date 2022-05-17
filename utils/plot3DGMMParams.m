function [hf] = plot3DGMMParams(GMM)        
    % GMM 
    Priors    = GMM.Priors;
    Mu        = GMM.Mu;
    Sigma     = GMM.Sigma;    
    K         = length(Priors);
    
    % Clustered Sigmas GMM
    colors = vivid(K);
    colors = hsv(K);
    colors = jet(K);
    
    for k=1:K 
        
        [V,D]=eig(Sigma(:,:,k));
        scale = 10;
        [x,y,z] = created3DgaussianEllipsoid(Mu(:,k),V,D, scale);

        % This makes the ellipsoids beautiful  
        hf = surf(x, y, z,'FaceColor',colors(k,:),'FaceAlpha', 0.25, 'FaceLighting','phong','EdgeColor','none');        
%         camlight
    end            
end