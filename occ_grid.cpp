#include "grid.h"
//cells per lato
#define cells 100;

	Grid(double size){
		size_grid = size;
	}

	void new_occ_grid(double xr, double yr){
		for (int i = 0; i < cells; i++)
		{
			for (int j = 0; j < cells; j++)
			{
				occ_grid[i][j] = 0;
			}
		}
		start_cell_x = xr - size_grid / 2;
		start_cell_y = yr - size_grid / 2;
	}

	void new_occ(double xo, double yo){
		if ((xo < start_cell_x) || (xo > start_cell_x + size_grid) || (yo < start_cell_y) || (yo> start_cell_y + size_grid))
			return;
		int index1 = (int)((xo - start_cell_x) / (size_grid / cells));
		int index2 = (int)((yo - start_cell_y) / (size_grid / cells));
		occ_grid[index1][index2] ++;
		return;
	}

	void decade_occ(){
		for (int i = 0; i < cells; i++)
		{
			for (int j = 0; j < cells; j++)
			{
				if (occ_grid[i][j] > 0)
					occ_grid[i][j]--;
			}
		}
	}

	void enlist_occ(double zr){
		Obstacles.clear()
			for (int i = 0; i < cells; i++)
			{
				for (int j = 0; j < cells; j++)
				{
					if (occ_grid[i][j] > 0){
						double xo = start_cell_x + (size_grid / cells) *i;
						double yo = start_cell_y + (size_grid / cells) *j;
						double zo = zr;
						Obstacle *obs = new Obstacle(xo, yo, zo, grid[i][j], (size_grid / cells));
						Obstacles.push_back(obs);
					}
				}
			}
	}



	//Questa non è detto che ci serva, meglio non utilizzarla per ora che non si sa mai...
	void update_occ(double xr, double yr){
		if ((xr > (start_cell_x + size_grid * 2 / 3)) || (xr < (start_cell_x + size_grid * 1 / 3)){
			shift_x(xr);
		}
		if ((xr >(start_cell_r + size_grid * 2 / 3)) || (xr < (start_cell_r + size_grid * 1 / 3)){
			shift_yr(yr);
		}
	}

	void shift_x(double xr){
		//spostamento rispetto alla griglia precedente
		double shift = xr - start_cell_x + size_grid / 2;
		//numero di caselle da spostare (positivo/negativo)
		int num_shift = shift / (size_grid / cells);
		for (int i = 0; i < cells; i++)
		{
			for (int j = 0; j < cells; j++)
			{
				if ((num_shift > 0) || (i + num_shift >= cells)){
					occ_grid[i][j] = occ_grid[i + num_shif][j];
				}
				if ((num_shift < 0) || (i - num_shift < 0)){
					occ_grid[cells - i][j] = occ_grid[cells - i - num_shif][j];
				}
			}
		}
	}

	void shift_y(double yr){
		//spostamento rispetto alla griglia precedente
		double shift = yr - start_cell_y + size_grid / 2;
		//numero di caselle da spostare (positivo/negativo)
		int num_shift = shift / (size_grid / cells);
		for (int i = 0; i < cells; i++)
		{
			for (int j = 0; j < cells; j++)
			{
				if ((num_shift > 0) || (j + num_shift >= cells)){
					occ_grid[i][j] = occ_grid[i][j + num_shif];
				}
				if ((num_shift < 0) || (j - num_shift < 0)){
					occ_grid[i][cells - j] = occ_grid[i][cells - j - num_shif];
				}
			}
		}
	}
};

