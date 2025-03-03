shader_type spatial;

void vertex() {
	// Called for every vertex the material is visible on.
}

void fragment() {
	float scale = 0.02;
	float thickness = 0.1;
	vec2 gradient_grid = mod(UV, scale) / scale;
	float grid = float(gradient_grid.r > thickness && gradient_grid.g > thickness);
	ALBEDO = mix(vec3(0.3), vec3(0.6,0.59,0.58), grid);
}
