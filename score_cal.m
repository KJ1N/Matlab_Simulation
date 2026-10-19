function [score_total,scores]=score_cal(eigenvalues,E_eigenvalues,scored_index)
scores=-(eigenvalues./E_eigenvalues).^scored_index;
score_total=sum(scores);
end