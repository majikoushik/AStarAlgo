package com.astaralgo.environment;

/**
 * A simple object representing the Tiles in the
 * environment.
 * DO NOT MODIFY.
 */
public class Tile {
	private TileStatus status;
	
	public Tile(TileStatus status) {
		this.status = status;
	}
	protected void highLightTile() {
		status = TileStatus.DIRTY;
	  }
	
	public TileStatus getStatus() { return status; }
	public String toString() { return ""+status.toString().charAt(0); }
}
