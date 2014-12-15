
//cells per lato
#define cells 100;

class Grid{
	//dimensione lato griglia
	double size_grid;
	//position of cell [0][0]
	double start_cell_x;
	double start_cell_y;
	int occ_grid[cells][cells];

public:
	Grid(double size);
	void new_occ_grid(double xr, double yr);
	void new_occ(double xo, double yo);
	void decade_occ();
	void enlist_occ(double zr);
	//Questa non è detto che ci serva, meglio non utilizzarla per ora che non si sa mai...
	void update_occ(double xr, double yr);
	void shift_x(double xr);
	void shift_y(double yr);
};

